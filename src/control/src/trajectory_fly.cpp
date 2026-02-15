// #include <rclcpp/rclcpp.hpp>
// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_status.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_odometry.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <Eigen/Dense>
// #include <vector>
// #include <chrono>
// #include <cmath>

// using namespace std::chrono_literals;

// class LinearTrajectoryController : public rclcpp::Node
// {
// public:
//     LinearTrajectoryController() : Node("linear_landing_controller"), clock_(RCL_STEADY_TIME)
//     {
//         auto qos = rclcpp::SensorDataQoS();

//         vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
//             "/fmu/out/vehicle_status", qos,
//             std::bind(&LinearTrajectoryController::vehicle_status_callback, this, std::placeholders::_1));

//         vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
//             "/fmu/out/vehicle_odometry", qos,
//             std::bind(&LinearTrajectoryController::odometry_callback, this, std::placeholders::_1));

//         offboard_control_mode_pub_ =
//             create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 100);

//         trajectory_setpoint_pub_ =
//             create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 100);

//         vehicle_command_pub_ =
//             create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 100);

//         timer_ = create_wall_timer(100ms, std::bind(&LinearTrajectoryController::cmdloop_callback, this));

//         square_waypoints_ = {
//             {0.0f, 0.0f, -2.0f},
//             {3.0f, 0.0f, -2.0f},
//             {3.0f, 3.0f, -2.0f},
//             {0.0f, 3.0f, -2.0f},
//             {0.0f, 0.0f, -2.0f}
//         };
//     }

// private:
//     rclcpp::Clock clock_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
//     rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
//     rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

//     rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
//     rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

//     Eigen::Vector3f current_position_ = Eigen::Vector3f::Zero();
//     std::vector<Eigen::Vector3f> square_waypoints_;
//     std::vector<Eigen::Vector3f> trajectory_points_;

//     size_t waypoint_index_ = 0;
//     size_t traj_index_ = 0;

//     std::string state_ = "INIT";

//     uint8_t arm_state_ = 0;

//     float current_yaw_ = M_PI_2;

//     /* ---------------- STATE LOGGER ---------------- */
//     void set_state(const std::string &new_state)
//     {
//         if (state_ != new_state)
//         {
//             RCLCPP_INFO(get_logger(), "STATE CHANGE: %s -> %s",
//                         state_.c_str(), new_state.c_str());
//             state_ = new_state;
//         }
//     }

//     /* ---------------- YAW NORMALIZATION ---------------- */
//     float normalize_yaw(float yaw)
//     {
//         while (yaw > M_PI) yaw -= 2.0f * M_PI;
//         while (yaw < -M_PI) yaw += 2.0f * M_PI;
//         return yaw;
//     }

//     void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
//     {
//         current_position_ = Eigen::Vector3f(msg->position[0], msg->position[1], msg->position[2]);
//     }

//     void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
//     {
//         arm_state_ = msg->arming_state;
//     }

//     /* ---------------- MAIN FSM ---------------- */
//     void cmdloop_callback()
//     {
//         publish_offboard_control_mode();

//         if (state_ == "INIT")
//         {
//             publish_setpoint(0, 0, -0.5, current_yaw_);
//             set_state("OFFBOARD");
//         }
//         else if (state_ == "OFFBOARD")
//         {
//             publish_setpoint(0, 0, -2, current_yaw_);
//             set_offboard_mode();
//             arm();
//             set_state("TAKEOFF");
//         }
//         else if (state_ == "TAKEOFF")
//         {
//             publish_setpoint(0, 0, -2, current_yaw_);

//             if (arm_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
//             {
//                 generate_linear_trajectory(current_position_, square_waypoints_[0]);
//                 traj_index_ = 0;
//                 set_state("MOVE_TO_START");
//             }
//         }
//         else if (state_ == "MOVE_TO_START")
//         {
//             if (execute_trajectory(current_yaw_))
//                 set_state("SQUARE_FLY");
//         }
//         else if (state_ == "SQUARE_FLY")
//         {
//             if (waypoint_index_ < square_waypoints_.size() - 1)
//             {
//                 Eigen::Vector3f start = current_position_;
//                 Eigen::Vector3f end = square_waypoints_[waypoint_index_ + 1];

//                 generate_linear_trajectory(start, end);
//                 waypoint_index_++;

//                 current_yaw_ = normalize_yaw(std::atan2(end.y()-start.y(), end.x()-start.x()));
//                 traj_index_ = 0;

//                 set_state("FLY_SEGMENT");
//             }
//             else
//                 set_state("LAND");
//         }
//         else if (state_ == "FLY_SEGMENT")
//         {
//             if (execute_trajectory(current_yaw_))
//                 set_state("SQUARE_FLY");
//         }
//         else if (state_ == "LAND")
//         {
//             land();
//         }
//     }

//     bool execute_trajectory(float yaw)
//     {
//         if (traj_index_ < trajectory_points_.size())
//         {
//             auto pt = trajectory_points_[traj_index_++];
//             publish_setpoint(pt.x(), pt.y(), pt.z(), yaw);
//             return false;
//         }
//         return true;
//     }

//     void generate_linear_trajectory(const Eigen::Vector3f &start, const Eigen::Vector3f &end)
//     {
//         trajectory_points_.clear();
//         const int N = 220;
//         for (int i = 0; i <= N; i++)
//         {
//             float t = float(i) / N;
//             trajectory_points_.push_back(start + t * (end - start));
//         }
//     }

//     void publish_offboard_control_mode()
//     {
//         px4_msgs::msg::OffboardControlMode msg;
//         msg.timestamp = clock_.now().nanoseconds() / 1000;
//         msg.position = true;
//         offboard_control_mode_pub_->publish(msg);
//     }

//     void publish_setpoint(float x, float y, float z, float yaw)
//     {
//         px4_msgs::msg::TrajectorySetpoint msg;
//         msg.timestamp = clock_.now().nanoseconds() / 1000;
//         msg.position = {x, y, z};
//         msg.yaw = normalize_yaw(yaw);
//         trajectory_setpoint_pub_->publish(msg);
//     }

//     void set_offboard_mode()
//     {
//         publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
//     }

//     void arm()
//     {
//         publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
//     }

//     void land()
//     {
//         publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
//     }

//     void publish_vehicle_command(uint16_t cmd, float p1 = 0, float p2 = 0)
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
//         vehicle_command_pub_->publish(msg);
//     }
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LinearTrajectoryController>());
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class LinearTrajectoryController : public rclcpp::Node
{
public:
    LinearTrajectoryController() : Node("linear_landing_controller"), clock_(RCL_STEADY_TIME)
    {
        auto qos = rclcpp::SensorDataQoS();

        vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            std::bind(&LinearTrajectoryController::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&LinearTrajectoryController::odometry_callback, this, std::placeholders::_1));

        offboard_control_mode_pub_ =
            create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 100);

        trajectory_setpoint_pub_ =
            create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 100);

        vehicle_command_pub_ =
            create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 100);

        timer_ = create_wall_timer(100ms, std::bind(&LinearTrajectoryController::cmdloop_callback, this));

        square_waypoints_ = {
            {0.0f, 0.0f, -2.0f},
            // {3.0f, 0.0f, -2.0f},
            // {3.0f, 3.0f, -2.0f},
            {0.0f, 3.0f, -2.0f},
            {0.0f, 0.0f, -2.0f}
        };
    }

private:
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

    Eigen::Vector3f current_position_ = Eigen::Vector3f::Zero();
    std::vector<Eigen::Vector3f> square_waypoints_;
    std::vector<Eigen::Vector3f> trajectory_points_;

    size_t waypoint_index_ = 0;
    size_t traj_index_ = 0;

    std::string state_ = "INIT";
    uint8_t arm_state_ = 0;

    float current_yaw_ = M_PI_2;

    /* -------- STATE LOGGER -------- */
    void set_state(const std::string &new_state)
    {
        if (state_ != new_state)
        {
            RCLCPP_INFO(get_logger(), "STATE CHANGE: %s -> %s",
                        state_.c_str(), new_state.c_str());
            state_ = new_state;
        }
    }

    /* -------- YAW NORMALIZATION -------- */
    float normalize_yaw(float yaw)
    {
        while (yaw > M_PI) yaw -= 2.0f * M_PI;
        while (yaw < -M_PI) yaw += 2.0f * M_PI;
        return yaw;
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_position_ = Eigen::Vector3f(msg->position[0], msg->position[1], msg->position[2]);
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        arm_state_ = msg->arming_state;
    }

    /* ---------------- MAIN FSM ---------------- */
    void cmdloop_callback()
    {
        publish_offboard_control_mode();

        const float takeoff_alt = -2.0f;

        if (state_ == "INIT")
        {
            publish_setpoint(0, 0, -0.5f, current_yaw_);
            set_state("OFFBOARD");
        }
        else if (state_ == "OFFBOARD")
        {
            publish_setpoint(0, 0, takeoff_alt, current_yaw_);
            set_offboard_mode();
            arm();
            set_state("TAKEOFF");
        }
        else if (state_ == "TAKEOFF")
        {
            publish_setpoint(0, 0, takeoff_alt, current_yaw_);

            if (arm_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
            {
                float alt_error = std::fabs(current_position_.z() - takeoff_alt);

                if (alt_error < 0.25f)
                {
                    RCLCPP_INFO(get_logger(), "Takeoff altitude reached");

                    generate_linear_trajectory(current_position_, square_waypoints_[0]);
                    traj_index_ = 0;
                    set_state("MOVE_TO_START");
                }
            }
        }
        else if (state_ == "MOVE_TO_START")
        {
            if (execute_trajectory(current_yaw_))
                set_state("SQUARE_FLY");
        }
        else if (state_ == "SQUARE_FLY")
        {
            if (waypoint_index_ < square_waypoints_.size() - 1)
            {
                Eigen::Vector3f start = current_position_;
                Eigen::Vector3f end = square_waypoints_[waypoint_index_ + 1];

                generate_linear_trajectory(start, end);
                waypoint_index_++;

                current_yaw_ = normalize_yaw(std::atan2(end.y()-start.y(), end.x()-start.x()));
                traj_index_ = 0;

                set_state("FLY_SEGMENT");
            }
            else
                set_state("LAND");
        }
        else if (state_ == "FLY_SEGMENT")
        {
            if (execute_trajectory(current_yaw_))
                set_state("SQUARE_FLY");
        }
        else if (state_ == "LAND")
        {
            land();
        }
    }

    bool execute_trajectory(float yaw)
    {
        if (traj_index_ < trajectory_points_.size())
        {
            auto pt = trajectory_points_[traj_index_++];
            publish_setpoint(pt.x(), pt.y(), pt.z(), yaw);
            return false;
        }
        return true;
    }

    void generate_linear_trajectory(const Eigen::Vector3f &start, const Eigen::Vector3f &end)
    {
        trajectory_points_.clear();
        const int N = 250;
        for (int i = 0; i <= N; i++)
        {
            float t = float(i) / N;
            trajectory_points_.push_back(start + t * (end - start));
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = clock_.now().nanoseconds() / 1000;
        msg.position = true;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_setpoint(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = clock_.now().nanoseconds() / 1000;
        msg.position = {x, y, z};
        msg.yaw = normalize_yaw(yaw);
        trajectory_setpoint_pub_->publish(msg);
    }

    void set_offboard_mode()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void arm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    }

    void land()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }

    void publish_vehicle_command(uint16_t cmd, float p1 = 0, float p2 = 0)
    {
        px4_msgs::msg::VehicleCommand msg;
        msg.timestamp = clock_.now().nanoseconds() / 1000;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.command = cmd;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearTrajectoryController>());
    rclcpp::shutdown();
    return 0;
}
