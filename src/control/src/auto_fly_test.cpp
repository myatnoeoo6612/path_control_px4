// #include <rclcpp/rclcpp.hpp>
// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_status.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_odometry.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// #include <Eigen/Dense>
// #include <chrono>
// #include <cmath>

// using namespace std::chrono_literals;

// class SimpleLandingController : public rclcpp::Node
// {
// public:
//     SimpleLandingController() : Node("auto_fly_test"), clock_(RCL_STEADY_TIME)
//     {
//         auto qos = rclcpp::SensorDataQoS();

//         odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
//             "/fmu/out/vehicle_odometry", qos,
//             std::bind(&SimpleLandingController::odom_callback, this, std::placeholders::_1));

//         status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
//             "/fmu/out/vehicle_status", qos,
//             std::bind(&SimpleLandingController::status_callback, this, std::placeholders::_1));

//         offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 100);
//         traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 100);
//         cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 100);

//         traj_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);

//         timer_ = create_wall_timer(100ms, std::bind(&SimpleLandingController::control_loop, this));
//     }

// private:
//     // ===================== ROS Vars =====================
//     rclcpp::Clock clock_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
//     rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

//     rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
//     rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
//     rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_marker_pub_;

//     // ===================== Drone State =====================
//     Eigen::Vector3f current_position_ = Eigen::Vector3f::Zero();
//     uint8_t arm_state_ = 0;
//     float current_yaw_ = 0.0f;  // ENU yaw (0 = East)

//     std::string state_ = "INIT";
//     rclcpp::Time hold_start_time_;
//     bool holding_ = false;

//     // ===================== Trajectory =====================
//     std::vector<Eigen::Vector3f> trajectory_points_;
//     int traj_index_ = 0;

//     // ============================== CALLBACKS =================================
//     void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
//     {
//         current_position_ = {msg->position[0], msg->position[1], msg->position[2]};
//     }

//     void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
//     {
//         arm_state_ = msg->arming_state;
//     }

//     // ============================= MAIN CONTROL ================================
//     void control_loop()
//     {
//         publish_offboard_mode();

//         if (state_ == "INIT")
//         {
//             publish_sp(0, 0, -0.5, current_yaw_);
//             state_ = "OFFBOARD";
//         }
//         else if (state_ == "OFFBOARD")
//         {
//             set_offboard_mode();
//             arm();
//             publish_sp(0, 0, -3.0, current_yaw_);
//             state_ = "TAKEOFF";
//         }
//         else if (state_ == "TAKEOFF")
//         {
//             publish_sp(0, 0, -5.0, current_yaw_);
//             if (position_reached({0, 0, -5.0}, 0.25))
//             {
//                 generate_linear_trajectory(
//                     current_position_,
//                     Eigen::Vector3f(-2.0, -2.0, -5.0));
//                 traj_index_ = 0;
//                 state_ = "MOVE_TO_POINT";
//             }
//         }
//         else if (state_ == "MOVE_TO_POINT")
//         {
//             execute_trajectory("HOLD");
//         }
//         else if (state_ == "HOLD")
//         {
//             run_hold_state("LAND");
//         }
//         // else if (state_ == "LAND")
//         // {
//         //     land();
//         // }
//     }

//     // =========================== TRAJECTORY ===========================
//     void execute_trajectory(const std::string &next_state)
//     {
//         if (traj_index_ < (int)trajectory_points_.size())
//         {
//             auto pt = trajectory_points_[traj_index_++];

//             float yaw = compute_heading_yaw(current_position_, pt);
//             current_yaw_ = yaw;

//             publish_sp(pt.x(), pt.y(), pt.z(), yaw);
//         }
//         else
//         {
//             holding_ = false;
//             hold_start_time_ = clock_.now();
//             state_ = next_state;
//         }
//     }

//     // =========================== HOLD ===========================
//     void run_hold_state(const std::string &next_state)
//     {
//         if (!holding_)
//         {
//             holding_ = true;
//             hold_start_time_ = clock_.now();
//         }

//         publish_sp(current_position_.x(),
//                    current_position_.y(),
//                    current_position_.z(),
//                    current_yaw_);

//         if ((clock_.now() - hold_start_time_).seconds() >= 3.0)
//         {
//             holding_ = false;
//             state_ = next_state;
//         }
//     }

//     // =========================== HELPERS ===========================
//     float compute_heading_yaw(const Eigen::Vector3f &from,
//                               const Eigen::Vector3f &to)
//     {
//         float dx = to.x() - from.x();  // East
//         float dy = to.y() - from.y();  // North
//         float yaw = std::atan2(dy, dx);
//         return normalize_yaw(yaw);
//     }

//     float normalize_yaw(float yaw)
//     {
//         while (yaw > M_PI) yaw -= 2.0f * M_PI;
//         while (yaw < -M_PI) yaw += 2.0f * M_PI;
//         return yaw;
//     }

//     bool position_reached(const Eigen::Vector3f &target, float tol)
//     {
//         return (current_position_ - target).norm() < tol;
//     }

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
//         msg.param1 = p1;
//         msg.param2 = p2;
//         msg.command = cmd;
//         msg.target_system = 1;
//         msg.target_component = 1;
//         msg.source_system = 1;
//         msg.source_component = 1;
//         msg.from_external = true;
//         cmd_pub_->publish(msg);
//     }

//     // =========================== TRAJECTORY GEN ===========================
//     void generate_linear_trajectory(const Eigen::Vector3f &start,
//                                     const Eigen::Vector3f &end)
//     {
//         const int N = 160;
//         trajectory_points_.clear();

//         visualization_msgs::msg::Marker line;
//         line.header.frame_id = "world";
//         line.header.stamp = get_clock()->now();
//         line.ns = "trajectory";
//         line.id = 1;
//         line.type = visualization_msgs::msg::Marker::LINE_STRIP;
//         line.scale.x = 0.05;
//         line.color.r = 1.0;
//         line.color.a = 1.0;

//         for (int i = 0; i <= N; i++)
//         {
//             float t = float(i) / N;
//             Eigen::Vector3f p = start + t * (end - start);
//             trajectory_points_.push_back(p);

//             geometry_msgs::msg::Point gp;
//             gp.x = p.x();
//             gp.y = p.y();
//             gp.z = p.z();
//             line.points.push_back(gp);
//         }

//         traj_marker_pub_->publish(line);
//     }
// };

// // ============================= MAIN =============================
// int main(int argc, char *argv[])
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
            "/fmu/in/offboard_control_mode", 100);

        traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 100);

        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 100);

        timer_ = create_wall_timer(
            100ms, std::bind(&SimpleLandingController::control_loop, this));

        RCLCPP_INFO(get_logger(), "PX4 Precision ArUco Landing (Trajectory) started");
    }

private:
    // ================= ROS =================
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    // ================= State =================
    Eigen::Vector3f drone_pos_{0,0,0};
    float drone_yaw_{0};

    Eigen::Vector3f aruco_pos_{0,0,0};
    float aruco_yaw_{0};
    bool aruco_valid_{false};

    // landing offset (XY only)
    Eigen::Vector3f landing_offset_{-0.0f, -0.1f, 0.0f};

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

    bool yaw_aligned(float tol_deg = 3.0f)
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
            publish_sp(0, 0, -0.5f, drone_yaw_);
            transition("OFFBOARD");
        }
        else if (state_ == "OFFBOARD")
        {
            set_offboard_mode();
            arm();
            publish_sp(0, 0, -3.0f, drone_yaw_);
            transition("TAKEOFF");
        }
        else if (state_ == "TAKEOFF")
        {
            publish_sp(0, 0, -5.0f, M_PI);
            if (position_reached({0,0,-5}, 0.3f))
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
            if (elapsed() > 5.0 && aruco_valid_)
                transition("ALIGN_YAW");
        }
        else if (state_ == "ALIGN_YAW")
        {
            publish_sp(drone_pos_.x(), drone_pos_.y(), drone_pos_.z(), aruco_yaw_);
            if (elapsed() > 5.0 && yaw_aligned())
            {
                // generate landing trajectory ONCE
                Eigen::Vector3f landing_target =
                    aruco_pos_ + landing_offset_;
                landing_target.z() = -2.0;

                generate_trajectory(drone_pos_, landing_target);
                transition("MOVE_TO_ARUCO_TRAJ");
            }
        }
        else if (state_ == "MOVE_TO_ARUCO_TRAJ")
        {
            follow_trajectory("PRECISION_LAND");
            print_debug();
        }
        else if (state_ == "PRECISION_LAND")
        {
            // precision_land();
            land();
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
        m.position = {x,y,z};
        m.yaw = yaw;
        traj_pub_->publish(m);
    }

    void precision_land()
    {
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_PRECLAND);
    }

    void land()
    {
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
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

    // ================= Trajectory =================
    void generate_trajectory(const Eigen::Vector3f &start,
                             const Eigen::Vector3f &end)
    {
        trajectory_.clear();
        traj_idx_ = 0;

        constexpr int N = 200;
        for (int i = 0; i <= N; i++)
        {
            float t = float(i) / N;
            trajectory_.push_back(start + t * (end - start));
        }
    }

    void follow_trajectory(const std::string &next)
    {
        if (traj_idx_ < (int)trajectory_.size())
        {
            auto &p = trajectory_[traj_idx_++];
            publish_sp(p.x(), p.y(), p.z(), aruco_yaw_);
        }
        else
        {
            transition(next);
        }
    }

    // ================= Utils =================
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

    void print_debug()
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "\n[PRECISION LANDING TRAJ]\n"
            "Drone XY : [%.2f %.2f]\n"
            "Aruco XY : [%.2f %.2f]\n"
            "Drone Yaw: %.1f deg | Aruco Yaw: %.1f deg\n",
            drone_pos_.x(), drone_pos_.y(),
            aruco_pos_.x(), aruco_pos_.y(),
            drone_yaw_ * 180.0 / M_PI,
            aruco_yaw_ * 180.0 / M_PI);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLandingController>());
    rclcpp::shutdown();
    return 0;
}
