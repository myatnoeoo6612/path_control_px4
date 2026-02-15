#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
// #include <px4_msgs/msg/vehicle_visual_odometry.hpp>   // ADDED

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class LinearTrajectoryController : public rclcpp::Node
{
public:
    LinearTrajectoryController()
        : Node("linear_landing_controller"),
          clock_(RCL_STEADY_TIME)
    {
        auto qos = rclcpp::SensorDataQoS();

        vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            std::bind(&LinearTrajectoryController::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&LinearTrajectoryController::odometry_callback, this, std::placeholders::_1));

        aruco_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco/pose_world", qos,
            std::bind(&LinearTrajectoryController::aruco_callback, this, std::placeholders::_1));

        offboard_control_mode_pub_ =
            create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 100);

        trajectory_setpoint_pub_ =
            create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", 100);

        vehicle_command_pub_ =
            create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", 100);

        vis_odom_pub_ =
            create_publisher<px4_msgs::msg::VehicleOdometry>(
                "/fmu/in/vehicle_visual_odometry", 100);


        timer_ = create_wall_timer(
            100ms, std::bind(&LinearTrajectoryController::cmdloop_callback, this));

        square_waypoints_ = {
            {0.0f, 0.0f, -4.0f},
            // {2.0f, 0.0f, -4.0f},
            // {2.0f, 2.0f, -4.0f},
            // {0.0f, 2.0f, -4.0f},
            // {0.0f, 0.0f, -4.0f},
            {-1.8f, -1.8f, -4.0f}};

        set_state("INIT");
    }

private:
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vis_odom_pub_;


    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;

    Eigen::Vector3f current_position_{0, 0, 0};
    Eigen::Vector3f aruco_position_{0, 0, 0};

    Eigen::Vector3f frozen_aruco_position_{0, 0, 0};
    float frozen_aruco_yaw_{0.0f};
    bool aruco_frozen_{false};

    float current_yaw_{0.0f};
    float aruco_yaw_{0.0f};
    float segment_yaw_{0.0f};

    bool aruco_valid_{false};
    uint8_t arm_state_{0};

    std::vector<Eigen::Vector3f> square_waypoints_;
    std::vector<Eigen::Vector3f> trajectory_points_;

    size_t waypoint_index_{0};
    size_t traj_index_{0};

    std::string state_{"INIT"};

    rclcpp::Time hold_start_time_;
    bool holding_{false};

    constexpr static float YAW_TOL_DEG = 5.0f;
    constexpr static float YAW_WAIT_SEC = 1.5f;

    float normalize_angle(float a)
    {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }

    bool yaw_within_tolerance()
    {
        float err = normalize_angle(current_yaw_ - aruco_yaw_);
        return std::abs(err) < YAW_TOL_DEG * M_PI / 180.0f;
    }

    bool xy_within_tolerance(const Eigen::Vector3f &target, float tol = 0.13f)
    {
        return std::abs(current_position_.x() - target.x()) < tol &&
               std::abs(current_position_.y() - target.y()) < tol;
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_position_ = {msg->position[0], msg->position[1], msg->position[2]};

        float qw = msg->q[0];
        float qx = msg->q[1];
        float qy = msg->q[2];
        float qz = msg->q[3];

        current_yaw_ = normalize_angle(
            std::atan2(2.0f * (qw * qz + qx * qy),
                       1.0f - 2.0f * (qy * qy + qz * qz)));
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        arm_state_ = msg->arming_state;
    }

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (aruco_frozen_) return;

        aruco_position_ = {
            (float)msg->pose.position.x,
            (float)msg->pose.position.y,
            (float)msg->pose.position.z};

        float yaw_enu = std::atan2(
            2.0 * (msg->pose.orientation.w * msg->pose.orientation.z),
            1.0 - 2.0 * (msg->pose.orientation.z * msg->pose.orientation.z));

        aruco_yaw_ = normalize_angle(-yaw_enu);
        aruco_valid_ = true;

px4_msgs::msg::VehicleOdometry vo{};
vo.timestamp = clock_.now().nanoseconds() / 1000;

vo.position = {
    (float)msg->pose.position.x,
    (float)msg->pose.position.y,
    (float)msg->pose.position.z};

vo.q = {
    (float)msg->pose.orientation.w,
    (float)msg->pose.orientation.x,
    (float)msg->pose.orientation.y,
    (float)msg->pose.orientation.z};


        vis_odom_pub_->publish(vo);
    }

    void cmdloop_callback()
    {
        publish_offboard_control_mode();

        if (state_ == "INIT")
        {
            publish_setpoint(0, 0, -0.8, 1.57);
            set_state("OFFBOARD");
        }
        else if (state_ == "OFFBOARD")
        {
            publish_setpoint(0, 0, -3, 1.57);
            set_offboard_mode();
            arm();
            set_state("TAKEOFF");
        }
        else if (state_ == "TAKEOFF")
        {
            publish_setpoint(0, 0, -3, 1.57);
            if (arm_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
            {
                generate_linear_trajectory(current_position_, square_waypoints_[0]);
                traj_index_ = 0;
                set_state("MOVE_TO_START");
            }
        }
        else if (state_ == "MOVE_TO_START")
        {
            execute_trajectory("START_HOLD", 1.57f);
        }
        else if (state_ == "START_HOLD")
        {
            run_hold_state("SQUARE_FLY");
        }
        else if (state_ == "SQUARE_FLY")
        {
            if (waypoint_index_ < square_waypoints_.size() - 1)
            {
                Eigen::Vector3f start = current_position_;
                Eigen::Vector3f end = square_waypoints_[++waypoint_index_];

                generate_linear_trajectory(start, end);

                float dx = end.x() - start.x();
                float dy = end.y() - start.y();
                segment_yaw_ = std::atan2(dy, dx);

                traj_index_ = 0;
                set_state("FLY_SEGMENT");
            }
            else
            {
                set_state("RETURN_YAW_ALIGN");
            }
        }
        else if (state_ == "FLY_SEGMENT")
        {
            execute_trajectory("CORNER_HOLD", segment_yaw_);
        }
        else if (state_ == "CORNER_HOLD")
        {
            run_hold_state("SQUARE_FLY");
        }
        // ===== LANDING PART REMAINS ORIGINAL =====
        else if (state_ == "RETURN_YAW_ALIGN")
        {
            if (!aruco_valid_) return;

            publish_setpoint(
                current_position_.x(),
                current_position_.y(),
                current_position_.z(),
                aruco_yaw_);

            if (!holding_)
            {
                holding_ = true;
                hold_start_time_ = clock_.now();
            }

            if ((clock_.now() - hold_start_time_).seconds() >= YAW_WAIT_SEC &&
                yaw_within_tolerance())
            {
                frozen_aruco_position_ = aruco_position_;
                frozen_aruco_position_.x() -= 0.0; //0.15
                frozen_aruco_position_.y() -= 0.0; //0.3
                frozen_aruco_yaw_ = aruco_yaw_;
                aruco_frozen_ = true;

                generate_linear_trajectory(
                    current_position_,
                    frozen_aruco_position_);

                traj_index_ = 0;
                holding_ = false;
                set_state("RETURN_MOVE_TO_ARUCO");
            }
        }
        else if (state_ == "RETURN_MOVE_TO_ARUCO")
        {
            if (traj_index_ < trajectory_points_.size())
            {
                auto &p = trajectory_points_[traj_index_++];
                publish_setpoint(p.x(), p.y(), -4.0, frozen_aruco_yaw_);
                return;
            }

            publish_setpoint(
                frozen_aruco_position_.x(),
                frozen_aruco_position_.y(),
                -4.0,
                frozen_aruco_yaw_);

            if (!holding_)
            {
                holding_ = true;
                hold_start_time_ = clock_.now();
            }

            if (xy_within_tolerance(frozen_aruco_position_, 0.05f))
            {
                if ((clock_.now() - hold_start_time_).seconds() > 0.0)
                {
                    set_state("LAND");
                }
            }
            else
            {
                hold_start_time_ = clock_.now();
            }
        }
        else if (state_ == "LAND")
        {
            precision_land();
        }
    }

    void set_state(const std::string &s)
    {
        if (state_ != s)
        {
            RCLCPP_WARN(get_logger(), "STATE → %s", s.c_str());
            state_ = s;
        }
    }

    void run_hold_state(const std::string &next)
    {
        if (!holding_)
        {
            holding_ = true;
            hold_start_time_ = clock_.now();
        }

        publish_setpoint(
            current_position_.x(),
            current_position_.y(),
            current_position_.z(),
            current_yaw_);

        if ((clock_.now() - hold_start_time_).seconds() > 3.0)
        {
            holding_ = false;
            set_state(next);
        }
    }

    void generate_linear_trajectory(const Eigen::Vector3f &a,
                                    const Eigen::Vector3f &b)
    {
        trajectory_points_.clear();
        const int N = 250;
        for (int i = 0; i <= N; i++)
            trajectory_points_.push_back(a + (float(i) / N) * (b - a));
    }

    void execute_trajectory(const std::string &next, float yaw)
    {
        if (traj_index_ < trajectory_points_.size())
        {
            auto &p = trajectory_points_[traj_index_++];
            publish_setpoint(p.x(), p.y(), p.z(), yaw);
        }
        else
        {
            set_state(next);
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode m{};
        m.timestamp = clock_.now().nanoseconds() / 1000;
        m.position = true;
        offboard_control_mode_pub_->publish(m);
    }

    void publish_setpoint(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint m{};
        m.timestamp = clock_.now().nanoseconds() / 1000;
        m.position = {x, y, z};
        m.yaw = yaw;
        trajectory_setpoint_pub_->publish(m);
    }

    void set_offboard_mode()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void arm()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    }

    void precision_land()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_PRECLAND);
    }

    void publish_vehicle_command(uint16_t cmd, float p1 = 0, float p2 = 0)
    {
        px4_msgs::msg::VehicleCommand m{};
        m.timestamp = clock_.now().nanoseconds() / 1000;
        m.command = cmd;
        m.param1 = p1;
        m.param2 = p2;
        m.target_system = 1;
        m.target_component = 1;
        m.source_system = 1;
        m.source_component = 1;
        m.from_external = true;
        vehicle_command_pub_->publish(m);
    }
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearTrajectoryController>());
    rclcpp::shutdown();
    return 0;
}