// #include <rclcpp/rclcpp.hpp>

// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_odometry.hpp>

// #include <geometry_msgs/msg/pose_stamped.hpp>

// #include <Eigen/Dense>
// #include <chrono>
// #include <cmath>
// #include <vector>

// using namespace std::chrono_literals;

// class SimpleLandingController : public rclcpp::Node
// {
// public:
//     SimpleLandingController()
//         : Node("precision_aruco_landing"),
//           clock_(RCL_STEADY_TIME)
//     {
//         auto qos = rclcpp::SensorDataQoS();

//         odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
//             "/fmu/out/vehicle_odometry", qos,
//             std::bind(&SimpleLandingController::odom_callback, this, std::placeholders::_1));

//         aruco_world_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/aruco/pose_world", qos,
//             std::bind(&SimpleLandingController::aruco_world_callback, this, std::placeholders::_1));

//         offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
//             "/fmu/in/offboard_control_mode", 10);

//         traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
//             "/fmu/in/trajectory_setpoint", 10);

//         cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
//             "/fmu/in/vehicle_command", 10);

//         timer_ = create_wall_timer(
//             100ms, std::bind(&SimpleLandingController::control_loop, this));

//         RCLCPP_INFO(get_logger(), "Precision ArUco Landing Controller started");
//     }

// private:
//     // ================= ROS =================
//     rclcpp::Clock clock_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_world_sub_;

//     rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
//     rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
//     rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

//     // ================= State =================
//     Eigen::Vector3f drone_pos_{0.f, 0.f, 0.f};
//     float drone_yaw_{0.f};

//     Eigen::Vector3f aruco_world_pos_{0.f, 0.f, 0.f};
//     float aruco_world_yaw_{0.f};
//     bool aruco_world_valid_{false};

//     Eigen::Vector3f target_world_{0.f, 0.f, 0.f};
//     float target_yaw_{0.f};

//     std::string state_{"INIT"};
//     rclcpp::Time state_start_;

//     std::vector<Eigen::Vector3f> trajectory_;
//     int traj_idx_{0};

//     // ================= Callbacks =================
//     void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
//     {
//         drone_pos_ = {
//             static_cast<float>(msg->position[0]),
//             static_cast<float>(msg->position[1]),
//             static_cast<float>(msg->position[2])
//         };

//         drone_yaw_ = 2.0f * std::atan2(msg->q[3], msg->q[0]);
//     }

//     void aruco_world_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//     {
//         aruco_world_pos_ = Eigen::Vector3f(
//             static_cast<float>(msg->pose.position.x),
//             static_cast<float>(msg->pose.position.y),
//             static_cast<float>(msg->pose.position.z)
//         );

//         aruco_world_yaw_ = std::atan2(
//             2.0 * (msg->pose.orientation.w * msg->pose.orientation.z),
//             1.0 - 2.0 * msg->pose.orientation.z * msg->pose.orientation.z
//         );

//         aruco_world_valid_ = true;
//     }

//     // ================= Control Loop =================
//     void control_loop()
//     {
//         publish_offboard_mode();

//         if (state_ == "INIT")
//         {
//             publish_sp(0, 0, -1.0f, drone_yaw_);
//             transition("OFFBOARD");
//         }
//         else if (state_ == "OFFBOARD")
//         {
//             set_offboard_mode();
//             arm();
//             publish_sp(0, 0, -3.0f, drone_yaw_);
//             transition("TAKEOFF");
//         }
//         else if (state_ == "TAKEOFF")
//         {
//             publish_sp(0, 0, -5.0f, drone_yaw_);
//             if (position_reached({0, 0, -5}, 0.3f))
//             {
//                 generate_linear_trajectory(drone_pos_, {-1.8f, -1.8f, -4.0f});
//                 traj_idx_ = 0;
//                 transition("MOVE_TO_PRELAND");
//             }
//         }
//         else if (state_ == "MOVE_TO_PRELAND")
//         {
//             execute_trajectory("HOLD");
//         }
//         else if (state_ == "HOLD")
//         {
//             hold_position();
//             if (elapsed() > 2.0 && aruco_world_valid_)
//                 transition("ALIGN_YAW");
//         }
//         else if (state_ == "ALIGN_YAW")
//         {
//             target_yaw_ = aruco_world_yaw_;
//             publish_sp(drone_pos_.x(), drone_pos_.y(), drone_pos_.z(), target_yaw_);

//             if (std::abs(angle_error(drone_yaw_, target_yaw_)) < deg2rad(2.0f))
//                 transition("MOVE_TO_ARUCO");
//         }
//         else if (state_ == "MOVE_TO_ARUCO")
//         {
//             target_world_ = aruco_world_pos_;
//             target_yaw_ = aruco_world_yaw_;

//             // Keep altitude, move only XY
//             publish_sp(
//                 target_world_.x(),
//                 target_world_.y(),
//                 drone_pos_.z(),
//                 target_yaw_);

//             if (position_reached(
//                     {target_world_.x(), target_world_.y(), drone_pos_.z()}, 0.12f))
//             {
//                 transition("LAND");
//             }
//         }
//         else if (state_ == "LAND")
//         {
//             land();
//         }
//     }

//     // ================= Helpers =================
//     void transition(const std::string &next)
//     {
//         state_ = next;
//         state_start_ = clock_.now();
//         RCLCPP_WARN(get_logger(), "==> STATE: %s", state_.c_str());
//     }

//     double elapsed()
//     {
//         return (clock_.now() - state_start_).seconds();
//     }

//     void hold_position()
//     {
//         publish_sp(drone_pos_.x(), drone_pos_.y(), drone_pos_.z(), drone_yaw_);
//     }

//     bool position_reached(const Eigen::Vector3f &target, float tol)
//     {
//         return (drone_pos_ - target).norm() < tol;
//     }

//     float angle_error(float a, float b)
//     {
//         float e = a - b;
//         while (e > M_PI) e -= 2 * M_PI;
//         while (e < -M_PI) e += 2 * M_PI;
//         return e;
//     }

//     float deg2rad(float d) { return d * M_PI / 180.0f; }

//     // ================= PX4 =================
//     void publish_offboard_mode()
//     {
//         px4_msgs::msg::OffboardControlMode msg;
//         msg.timestamp = clock_.now().nanoseconds() / 1000;
//         msg.position = true;
//         offboard_pub_->publish(msg);
//     }

//     void publish_sp(float x, float y, float z, float yaw)
//     {
//         px4_msgs::msg::TrajectorySetpoint msg;
//         msg.timestamp = clock_.now().nanoseconds() / 1000;
//         msg.position = {x, y, z};
//         msg.yaw = yaw;
//         traj_pub_->publish(msg);
//     }

//     void set_offboard_mode()
//     {
//         send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
//     }

//     void arm()
//     {
//         send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
//     }

//     void land()
//     {
//         send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
//     }

//     void send_cmd(uint16_t cmd, float p1 = 0, float p2 = 0)
//     {
//         px4_msgs::msg::VehicleCommand msg;
//         msg.timestamp = clock_.now().nanoseconds() / 1000;
//         msg.command = cmd;
//         msg.param1 = p1;
//         msg.param2 = p2;
//         msg.target_system = 1;
//         msg.target_component = 1;
//         msg.source_system = 1;
//         msg.source_component = 1;
//         msg.from_external = true;
//         cmd_pub_->publish(msg);
//     }

//     void generate_linear_trajectory(const Eigen::Vector3f &start,
//                                     const Eigen::Vector3f &end)
//     {
//         trajectory_.clear();
//         const int N = 80;
//         for (int i = 0; i <= N; i++)
//         {
//             float t = static_cast<float>(i) / N;
//             trajectory_.push_back(start + t * (end - start));
//         }
//     }

//     void execute_trajectory(const std::string &next)
//     {
//         if (traj_idx_ < (int)trajectory_.size())
//         {
//             auto &p = trajectory_[traj_idx_++];
//             publish_sp(p.x(), p.y(), p.z(), drone_yaw_);
//         }
//         else
//         {
//             transition(next);
//         }
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SimpleLandingController>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class SimpleLandingController : public rclcpp::Node
{
public:
    SimpleLandingController()
        : Node("precision_aruco_landing"),
          clock_(RCL_STEADY_TIME)
    {
        auto qos = rclcpp::SensorDataQoS();

        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&SimpleLandingController::odom_callback, this, std::placeholders::_1));

        aruco_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco/pose_world", qos,
            std::bind(&SimpleLandingController::aruco_callback, this, std::placeholders::_1));

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        visual_odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);

        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        timer_ = create_wall_timer(
            100ms, std::bind(&SimpleLandingController::control_loop, this));

        RCLCPP_INFO(get_logger(), "PX4 Precision ArUco Landing Controller started");
    }

private:
    // ================= ROS =================
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    // ================= State =================
    Eigen::Vector3f drone_pos_{0, 0, 0};
    float drone_yaw_{0};

    Eigen::Vector3f aruco_pos_{0, 0, 0};
    float aruco_yaw_{0};
    bool aruco_valid_{false};

    Eigen::Vector3f landing_offset_{-0.2f, -0.2f, 0.0f};

    std::vector<Eigen::Vector3f> trajectory_;
    int traj_idx_{0};

    std::string state_{"INIT"};
    rclcpp::Time state_start_;

    // ================= Callbacks =================
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        drone_pos_ = {
            (float)msg->position[0],
            (float)msg->position[1],
            (float)msg->position[2]
        };

        drone_yaw_ = normalize_yaw(
            2.0f * std::atan2(msg->q[3], msg->q[0]));
    }

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        aruco_pos_ = {
            (float)msg->pose.position.x,
            (float)msg->pose.position.y,
            (float)msg->pose.position.z
        };

        aruco_yaw_ = normalize_yaw(std::atan2(
            2.0 * msg->pose.orientation.w * msg->pose.orientation.z,
            1.0 - 2.0 * msg->pose.orientation.z * msg->pose.orientation.z));

        aruco_valid_ = true;
    }

    // ================= Helpers =================
    float normalize_yaw(float yaw)
    {
        while (yaw > M_PI) yaw -= 2.0f * M_PI;
        while (yaw < -M_PI) yaw += 2.0f * M_PI;
        return yaw;
    }

    bool yaw_aligned(float tol_deg = 5.0f)
    {
        float err = normalize_yaw(drone_yaw_ - aruco_yaw_);
        return std::abs(err) < tol_deg * M_PI / 180.0f;
    }

    double elapsed()
    {
        return (clock_.now() - state_start_).seconds();
    }

    // ================= Control Loop =================
    void control_loop()
    {
        publish_offboard_mode();

        if (state_ == "INIT")
        {
            publish_sp(0, 0, -0.5, drone_yaw_);
            transition("OFFBOARD");
        }
        else if (state_ == "OFFBOARD")
        {
            set_offboard_mode();
            arm();
            publish_sp(0, 0, -3, drone_yaw_);
            transition("TAKEOFF");
        }
        else if (state_ == "TAKEOFF")
        {
            publish_sp(0, 0, -5, drone_yaw_);
            if (position_reached({0, 0, -5}, 0.3))
            {
                generate_trajectory(drone_pos_, {-1.8f, -1.8f, -4.0f});
                transition("MOVE_TO_PRELAND");
            }
        }
        else if (state_ == "MOVE_TO_PRELAND")
        {
            follow_trajectory("PRELAND_WAIT");
        }
        else if (state_ == "PRELAND_WAIT")
        {
            hold();
            if (elapsed() > 5.0)
                transition("ALIGN_YAW");
        }
        else if (state_ == "ALIGN_YAW")
        {
            publish_sp(drone_pos_.x(), drone_pos_.y(), drone_pos_.z(), aruco_yaw_);
            if (elapsed() > 5.0 && yaw_aligned())
                transition("MOVE_TO_ARUCO");
        }
        else if (state_ == "MOVE_TO_ARUCO")
        {
            Eigen::Vector3f landing_target = aruco_pos_ + landing_offset_;

            publish_sp(
                landing_target.x() - 0.48,
                landing_target.y() - 0.48,
                drone_pos_.z(),
                aruco_yaw_);

            publish_visual_odometry();
            print_debug(landing_target);

            if ((drone_pos_.head<2>() - landing_target.head<2>()).norm() < 0.15f &&
                yaw_aligned())
            {
                transition("PRECISION_LAND");
            }
        }
        else if (state_ == "PRECISION_LAND")
        {
            precision_land();
        }
    }

    // ================= PX4 =================
    void publish_offboard_mode()
    {
        px4_msgs::msg::OffboardControlMode m{};
        m.timestamp = clock_.now().nanoseconds() / 1000;
        m.position = true;
        offboard_pub_->publish(m);
    }

    void publish_sp(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint m{};
        m.timestamp = clock_.now().nanoseconds() / 1000;
        m.position = {x, y, z};
        m.yaw = yaw;
        traj_pub_->publish(m);
    }

    void publish_visual_odometry()
    {
        if (!aruco_valid_) return;

        px4_msgs::msg::VehicleOdometry v{};
        v.timestamp = clock_.now().nanoseconds() / 1000;

        v.position = {
            aruco_pos_.x(),
            aruco_pos_.y(),
            aruco_pos_.z()
        };

        float h = aruco_yaw_ * 0.5f;
        v.q = {std::cos(h), 0.0f, 0.0f, std::sin(h)};

        v.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        v.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

        visual_odom_pub_->publish(v);
    }

    void precision_land()
    {
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_PRECLAND);
    }

    void set_offboard_mode()
    {
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void arm()
    {
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    }

    void send_cmd(uint16_t cmd, float p1 = 0, float p2 = 0)
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
        cmd_pub_->publish(m);
    }

    // ================= Utilities =================
    void transition(const std::string &s)
    {
        state_ = s;
        state_start_ = clock_.now();
        RCLCPP_WARN(get_logger(), "STATE → %s", s.c_str());
    }

    void hold()
    {
        publish_sp(drone_pos_.x(), drone_pos_.y(), drone_pos_.z(), drone_yaw_);
    }

    bool position_reached(const Eigen::Vector3f &p, float tol)
    {
        return (drone_pos_ - p).norm() < tol;
    }

    void generate_trajectory(const Eigen::Vector3f &a,
                             const Eigen::Vector3f &b)
    {
        trajectory_.clear();
        traj_idx_ = 0;
        for (int i = 0; i <= 60; i++)
            trajectory_.push_back(a + (float(i) / 60) * (b - a));
    }

    void follow_trajectory(const std::string &next)
    {
        if (traj_idx_ < (int)trajectory_.size())
        {
            auto &p = trajectory_[traj_idx_++];
            publish_sp(p.x(), p.y(), p.z(), drone_yaw_);
        }
        else
        {
            transition(next);
        }
    }

    void print_debug(const Eigen::Vector3f &landing_target)
    {
        float drone_yaw_deg = drone_yaw_ * 180.0f / M_PI;
        float aruco_yaw_deg = aruco_yaw_ * 180.0f / M_PI;
        float yaw_err_deg =
            normalize_yaw(drone_yaw_ - aruco_yaw_) * 180.0f / M_PI;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "\n[PRECISION LANDING DEBUG]\n"
            "Target XY : [%.2f , %.2f]\n"
            "Drone  XY : [%.2f , %.2f]\n"
            "Drone Yaw : %.1f deg\n"
            "Aruco Yaw : %.1f deg\n"
            "Yaw Error : %.1f deg\n",
            landing_target.x(), landing_target.y(),
            drone_pos_.x(), drone_pos_.y(),
            drone_yaw_deg,
            aruco_yaw_deg,
            yaw_err_deg
        );
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLandingController>());
    rclcpp::shutdown();
    return 0;
}
