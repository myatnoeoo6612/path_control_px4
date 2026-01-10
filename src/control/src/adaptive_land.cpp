#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

/* ================= FSM ================= */
enum class State {
    INIT = 0,
    OFFBOARD,
    TAKEOFF,
    MOVE_TO_PRELAND,
    PRELAND_WAIT,
    ALIGN_YAW,
    TRACK_IBVS,
    LAND
};

/* ================= KALMAN FILTER =================
   Constant velocity model in CAMERA FRAME
   x = [x, y, vx, vy]
*/
class CVKalman {
public:
    Eigen::Vector4f x;
    Eigen::Matrix4f P;
    Eigen::Matrix4f Q;
    Eigen::Matrix2f R;

    CVKalman() {
        x.setZero();
        P = Eigen::Matrix4f::Identity() * 0.2f;
        Q = Eigen::Matrix4f::Identity() * 0.01f;
        R = Eigen::Matrix2f::Identity() * 0.02f;
    }

    void predict(float dt) {
        Eigen::Matrix4f F;
        F << 1,0,dt,0,
             0,1,0,dt,
             0,0,1,0,
             0,0,0,1;
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::Vector2f &z) {
        Eigen::Matrix<float,2,4> H;
        H << 1,0,0,0,
             0,1,0,0;

        Eigen::Vector2f y = z - H * x;
        Eigen::Matrix2f S = H * P * H.transpose() + R;
        Eigen::Matrix<float,4,2> K = P * H.transpose() * S.inverse();

        x += K * y;
        P = (Eigen::Matrix4f::Identity() - K * H) * P;
    }
};

/* ================= MAIN NODE ================= */
class AdaptiveLandingIBVS : public rclcpp::Node {
public:
    AdaptiveLandingIBVS()
        : Node("adaptive_precision_landing_ibvs_kf"),
          clock_(RCL_STEADY_TIME)
    {
        auto qos = rclcpp::SensorDataQoS();

        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&AdaptiveLandingIBVS::odom_cb, this, std::placeholders::_1));

        aruco_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco/pose_world", qos,
            std::bind(&AdaptiveLandingIBVS::aruco_cb, this, std::placeholders::_1));

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 100);
        traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 100);
        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 100);

        timer_ = create_wall_timer(50ms,
            std::bind(&AdaptiveLandingIBVS::loop, this));

        RCLCPP_INFO(get_logger(), "IBVS + Kalman Precision Landing STARTED");
    }

private:
    /* ================= ROS ================= */
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    /* ================= STATE ================= */
    State state_{State::INIT};
    rclcpp::Time state_enter_t_;

    Eigen::Vector3f drone_pos_{0,0,0};
    float drone_yaw_{0};
    float ibvs_yaw_{0};

    /* ================= ARUCO (CAMERA FRAME) ================= */
    Eigen::Vector2f aruco_raw_{0,0};
    bool aruco_valid_{false};

    /* ================= FILTERING ================= */
    CVKalman kf_;
    rclcpp::Time last_kf_t_;
    bool kf_init_{false};

    /* ================= PRELAND ================= */
    Eigen::Vector3f preland_{-1.8f,-1.8f,-4.0f};
    std::vector<Eigen::Vector3f> traj_;
    size_t traj_i_{0};

    /* ================= CONTROL PARAMS ================= */
    const float kp_xy_      = 0.002f;
    const float v_xy_max_   = 0.1f;
    const float vz_descend_ = 0.1f;
    const float xy_gate_    = 0.20f;

    /* ================= CALLBACKS ================= */
    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        drone_pos_ = {(float)msg->position[0],
                      (float)msg->position[1],
                      (float)msg->position[2]};
        drone_yaw_ = normalize_yaw(2.f * std::atan2(msg->q[3], msg->q[0]));
    }

    void aruco_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        aruco_raw_ = {(float)msg->pose.position.x,
                      (float)msg->pose.position.y};
        aruco_valid_ = true;
    }

    /* ================= MAIN LOOP ================= */
    void loop() {
        publish_offboard();

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[STATE %d] Drone [%.2f %.2f %.2f]",
            (int)state_,
            drone_pos_.x(), drone_pos_.y(), drone_pos_.z());

        switch(state_) {
        case State::INIT:
            transition(State::OFFBOARD);
            break;

        case State::OFFBOARD:
            set_mode(); arm();
            transition(State::TAKEOFF);
            break;

        case State::TAKEOFF:
            publish_pos(0,0,-5,drone_yaw_);
            if (std::abs(drone_pos_.z()) > 4.8f) {
                gen_traj(drone_pos_, preland_);
                transition(State::MOVE_TO_PRELAND);
            }
            break;

        case State::MOVE_TO_PRELAND:
            if (follow_traj())
                transition(State::PRELAND_WAIT);
            break;

        case State::PRELAND_WAIT:
            publish_pos(preland_.x(), preland_.y(), preland_.z(), drone_yaw_);
            if ((clock_.now()-state_enter_t_).seconds() > 2.0)
                transition(State::ALIGN_YAW);
            break;

        case State::ALIGN_YAW:
            publish_pos(drone_pos_.x(), drone_pos_.y(),
                        drone_pos_.z(), drone_yaw_);
            if (aruco_valid_) {
                ibvs_yaw_ = drone_yaw_;
                last_kf_t_ = clock_.now();
                kf_init_ = false;
                transition(State::TRACK_IBVS);
            }
            break;

        case State::TRACK_IBVS:
            track_ibvs_kf();
            break;

        case State::LAND:
            send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            break;
        }
    }

    /* ================= IBVS + KALMAN ================= */
    void track_ibvs_kf() {
        float dt = (clock_.now() - last_kf_t_).seconds();
        last_kf_t_ = clock_.now();
        if (dt < 0.02f) dt = 0.2f;

        if (!kf_init_) {
            kf_.x << aruco_raw_.x(), aruco_raw_.y(), 0, 0;
            kf_init_ = true;
        }

        kf_.predict(dt);
        kf_.update(aruco_raw_);

        float ex = kf_.x(0);
        float ey = kf_.x(1);

        float xy_err = std::sqrt(ex*ex + ey*ey);

        // PX4 BODY FRAME IBVS
        float vx = std::clamp(+kp_xy_ * ey, -v_xy_max_, v_xy_max_);
        float vy = std::clamp(+kp_xy_ * ex, -v_xy_max_, v_xy_max_);
        float vz = (xy_err < xy_gate_) ? vz_descend_ : 0.0f;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 300,
            "[IBVS+KF] err=%.3f vx=%.2f vy=%.2f vz=%.2f",
            xy_err, vx, vy, vz);

        publish_vel(vx, vy, vz, ibvs_yaw_);

        if (drone_pos_.z() > -0.25f)
            transition(State::LAND);
    }

    /* ================= UTIL ================= */
    float normalize_yaw(float y){
        while(y>M_PI) y-=2*M_PI;
        while(y<-M_PI) y+=2*M_PI;
        return y;
    }

    void gen_traj(const Eigen::Vector3f&a,const Eigen::Vector3f&b){
        traj_.clear(); traj_i_=0;
        for(int i=0;i<=80;i++)
            traj_.push_back(a + float(i)/80*(b-a));
    }

    bool follow_traj(){
        if(traj_i_>=traj_.size()) return true;
        auto&p=traj_[traj_i_++];
        publish_pos(p.x(),p.y(),p.z(),drone_yaw_);
        return false;
    }

    void transition(State s){
        state_=s;
        state_enter_t_=clock_.now();
        RCLCPP_WARN(get_logger(),"STATE → %d",(int)s);
    }

    /* ================= PX4 ================= */
    void publish_offboard(){
        px4_msgs::msg::OffboardControlMode m{};
        m.timestamp=clock_.now().nanoseconds()/1000;
        m.velocity=true;
        offboard_pub_->publish(m);
    }

    void publish_pos(float x,float y,float z,float yaw){
        px4_msgs::msg::TrajectorySetpoint m{};
        m.timestamp=clock_.now().nanoseconds()/1000;
        m.position={x,y,z};
        m.yaw=yaw;
        traj_pub_->publish(m);
    }

    void publish_vel(float vx,float vy,float vz,float yaw){
        px4_msgs::msg::TrajectorySetpoint m{};
        m.timestamp=clock_.now().nanoseconds()/1000;
        m.velocity={vx,vy,vz};
        m.yaw=yaw;
        traj_pub_->publish(m);
    }

    void set_mode(){
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,6);
    }

    void arm(){
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,1);
    }

    void send_cmd(uint16_t cmd,float p1=0,float p2=0){
        px4_msgs::msg::VehicleCommand m{};
        m.timestamp=clock_.now().nanoseconds()/1000;
        m.command=cmd;
        m.param1=p1; m.param2=p2;
        m.target_system=1; m.target_component=1;
        m.source_system=1; m.source_component=1;
        m.from_external=true;
        cmd_pub_->publish(m);
    }
};

/* ================= MAIN ================= */
int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<AdaptiveLandingIBVS>());
    rclcpp::shutdown();
    return 0;
}
