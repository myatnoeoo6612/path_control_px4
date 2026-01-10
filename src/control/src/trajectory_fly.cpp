#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
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

        traj_marker_pub_ =
            create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 100);

        timer_ = create_wall_timer(100ms, std::bind(&LinearTrajectoryController::cmdloop_callback, this));

        // =========================== NEW WAYPOINTS (pattern flight) ===========================
        square_waypoints_ = {
            {0.0f, 0.0f, -4.0f}, 
            {18.0f, 0.0f, -4.0f}, 
            {18.0f, 11.0f, -4.0f}, 
            {11.0f, 11.0f, -4.0f},
            {11.0f, -11.0f, -4.0f}, 
            {4.0f, -11.0f, -4.0f},
            {4.0f, 11.0f, -4.0f},
            {-3.0f, 11.0f, -4.0f},
            {-3.0f, -11.0f, -4.0f},
            {-10.0f, -11.0f, -4.0f},
            {-10.0f, 11.0f, -4.0f},
            {-17.0f, 11.0f, -4.0f},
            {-17.0f, -11.0f, -4.0f},
            {-17.0f, 0.0f, -4.0f},
            {0.0f, 0.0f, -4.0f},
        };
    }

private:
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_marker_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

    Eigen::Vector3f current_position_ = Eigen::Vector3f::Zero();
    std::vector<Eigen::Vector3f> square_waypoints_;
    std::vector<Eigen::Vector3f> trajectory_points_;

    size_t waypoint_index_ = 0;
    size_t traj_index_ = 0;

    std::string state_ = "INIT";

    uint8_t arm_state_ = 0;
    rclcpp::Time hold_start_time_;
    bool holding_ = false;
    float current_yaw_ = 0.0;

    // ========================== PX4 ODOM CALLBACK ==========================
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_position_ = Eigen::Vector3f(msg->position[0], msg->position[1], msg->position[2]);
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        arm_state_ = msg->arming_state;
    }

    // ============================= MAIN STATE MACHINE ============================
    void cmdloop_callback()
    {
        publish_offboard_control_mode();

        if (state_ == "INIT")
        {
            publish_setpoint(0, 0, -1.5, 0);
            state_ = "OFFBOARD";
        }

        else if (state_ == "OFFBOARD")
        {
            publish_setpoint(0, 0, -4, 0);
            set_offboard_mode();
            arm();
            state_ = "TAKEOFF";
        }

        else if (state_ == "TAKEOFF")
        {
            publish_setpoint(0, 0, -4, 0);
            if (arm_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
            {

                generate_linear_trajectory(current_position_, square_waypoints_[0]);
                traj_index_ = 0;
                state_ = "MOVE_TO_START";
            }
        }

        else if (state_ == "MOVE_TO_START")
        {
            execute_trajectory("START_HOLD", 0);
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
                Eigen::Vector3f end = square_waypoints_[waypoint_index_ + 1];

                generate_linear_trajectory(start, end);
                waypoint_index_++;


                float dx = end.x() - start.x();
                float dy = end.y() - start.y();
                current_yaw_ = atan2(dy, dx);

                traj_index_ = 0;
                state_ = "FLY_SEGMENT";
            }
            else
            {
                state_ = "RETURN_HOME";
            }
        }

        else if (state_ == "FLY_SEGMENT")
        {
            execute_trajectory("CORNER_HOLD", current_yaw_);
        }

        else if (state_ == "CORNER_HOLD")
        {
            run_hold_state("SQUARE_FLY");
        }

        else if (state_ == "RETURN_HOME")
        {
            execute_trajectory("LAND", 0.0);
        }

        else if (state_ == "LAND")
        {
            land();
        }
    }

    // =========================== TRAJECTORY EXECUTION ===========================
    void execute_trajectory(const std::string &next_state, float yaw)
    {
        if (traj_index_ < trajectory_points_.size())
        {
            auto pt = trajectory_points_[traj_index_++];
            publish_setpoint(pt.x(), pt.y(), pt.z(), yaw);
        }
        else
        {
            holding_ = false;
            hold_start_time_ = clock_.now();
            state_ = next_state;
        }
    }

    // ============================= 3-SECOND HOLD ================================
    void run_hold_state(const std::string &next_state)
    {
        if (!holding_)
        {
            holding_ = true;
            hold_start_time_ = clock_.now();
        }

        float elapsed = (clock_.now() - hold_start_time_).seconds();
        publish_setpoint(current_position_.x(), current_position_.y(), current_position_.z(), current_yaw_);

        if (elapsed >= 3.0)
        {
            holding_ = false;
            state_ = next_state;
        }
    }

    // ========================= LERP TRAJECTORY GENERATION =======================
    void generate_linear_trajectory(const Eigen::Vector3f &start, const Eigen::Vector3f &end)
    {
        const int N = 160;
        trajectory_points_.clear();

        visualization_msgs::msg::Marker line;
        line.header.frame_id = "world";
        line.header.stamp = this->get_clock()->now();
        line.ns = "square";
        line.id = 1;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.scale.x = 0.05;
        line.color.r = 1.0;
        line.color.a = 1.0;

        for (int i = 0; i <= N; i++)
        {
            float t = float(i) / N;
            Eigen::Vector3f p = start + t * (end - start);
            trajectory_points_.push_back(p);

            geometry_msgs::msg::Point gp;
            gp.x = p.x();
            gp.y = p.y();
            gp.z = p.z();
            line.points.push_back(gp);
        }

        traj_marker_pub_->publish(line);
    }

    // =========================== PX4 COMMAND HELPERS ============================
    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = clock_.now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_setpoint(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = clock_.now().nanoseconds() / 1000;
        msg.position = {x, y, z};
        msg.yaw = yaw;
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

// =============================== MAIN =====================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearTrajectoryController>());
    rclcpp::shutdown();
    return 0;
}