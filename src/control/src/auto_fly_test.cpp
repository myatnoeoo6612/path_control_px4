#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class SimpleLandingController : public rclcpp::Node
{
public:
    SimpleLandingController() : Node("auto_fly_test"), clock_(RCL_STEADY_TIME)
    {
        auto qos = rclcpp::SensorDataQoS();

        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&SimpleLandingController::odom_callback, this, std::placeholders::_1));

        status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            std::bind(&SimpleLandingController::status_callback, this, std::placeholders::_1));

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 100);
        traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 100);
        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 100);

        timer_ = create_wall_timer(100ms, std::bind(&SimpleLandingController::control_loop, this));
    }

private:
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    Eigen::Vector3f current_pos_ = Eigen::Vector3f::Zero();
    uint8_t arm_state_ = 0;

    std::string state_ = "INIT";
    rclcpp::Time hold_start_time_;
    bool holding_ = false;

    // --------------------- Callbacks ------------------------
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_pos_ = {msg->position[0], msg->position[1], msg->position[2]};
    }

    void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        arm_state_ = msg->arming_state;
    }

    // ---------------------- Main State Machine -------------------------
    void control_loop()
    {
        publish_offboard_mode();

        if (state_ == "INIT")
        {
            publish_sp(0, 0, -0.5, 0);
            state_ = "OFFBOARD";
        }

        else if (state_ == "OFFBOARD")
        {
            set_offboard_mode();
            arm();
            publish_sp(0, 0, -3.0, 0);
            state_ = "TAKEOFF";
        }

        else if (state_ == "TAKEOFF")
        {
            publish_sp(0, 0, -3.0, 0);
            if (position_reached({0,0,-3.0}, 0.25))
            {
                state_ = "MOVE_TO_POINT";
            }
        }

        else if (state_ == "MOVE_TO_POINT")
        {
            publish_sp(0.0, 1.0, -3.0, 0);

            if (position_reached({0.0, 1.0, -3.0}, 0.25))
            {
                holding_ = false;
                state_ = "HOLD";
            }
        }

        else if (state_ == "HOLD")
        {
            if (!holding_)
            {
                holding_ = true;
                hold_start_time_ = clock_.now();
            }

            publish_sp(1.0, 0.0, -3.0, 0);

            if ((clock_.now() - hold_start_time_).seconds() >= 3.0)
            {
                state_ = "LAND";
            }
        }

        else if (state_ == "LAND")
        {
            land();
        }
    }

    // ---------------------- Helper Functions -------------------------
    bool position_reached(const Eigen::Vector3f &target, float tol)
    {
        return (current_pos_ - target).norm() < tol;
    }

    void publish_offboard_mode()
    {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = clock_.now().nanoseconds() / 1000;
        msg.position = true;
        offboard_pub_->publish(msg);
    }

    void publish_sp(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = clock_.now().nanoseconds() / 1000;
        msg.position = {x, y, z};
        msg.yaw = yaw;
        traj_pub_->publish(msg);
    }

    void set_offboard_mode()
    {
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void arm()
    {
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    }

    void land()
    {
        send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }

    void send_cmd(uint16_t cmd, float p1 = 0, float p2 = 0)
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
        cmd_pub_->publish(msg);
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLandingController>());
    rclcpp::shutdown();
    return 0;
}
