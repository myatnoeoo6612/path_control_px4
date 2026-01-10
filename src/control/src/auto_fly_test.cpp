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
#include <algorithm>

using namespace std::chrono_literals;

class RoundedCornerTrajectoryController : public rclcpp::Node
{
public:
    RoundedCornerTrajectoryController()
        : Node("rounded_corner_trajectory_controller"),
          clock_(RCL_STEADY_TIME)
    {
        auto qos = rclcpp::SensorDataQoS();

        vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            std::bind(&RoundedCornerTrajectoryController::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&RoundedCornerTrajectoryController::odometry_callback, this, std::placeholders::_1));

        offboard_control_mode_pub_ =
            create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_pub_ =
            create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ =
            create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", 10);

        traj_marker_pub_ =
            create_publisher<visualization_msgs::msg::Marker>(
                "trajectory_marker", 1);

        timer_ = create_wall_timer(
            100ms, std::bind(&RoundedCornerTrajectoryController::cmdloop_callback, this));

        // ===== Square waypoints (rounded later) =====
        waypoints_ = {
            {0.0f, 0.0f, -4.0f},
            {5.0f, 0.0f, -4.0f},
            {5.0f, 5.0f, -4.0f},
            {0.0f, 5.0f, -4.0f},
            {0.0f, 0.0f, -4.0f}
        };

        RCLCPP_INFO(get_logger(), "Rounded-corner trajectory controller (FIXED) started");
    }

private:
    // ================= CORE =================
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_marker_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

    // ================= STATE =================
    Eigen::Vector3f current_position_{0, 0, 0};
    float current_yaw_{0.0f};
    uint8_t arm_state_{0};

    const float POS_TOL = 0.15f;
    const float YAW_TOL = 10.0f * M_PI / 180.0f;

    std::string state_{"INIT"};

    std::vector<Eigen::Vector3f> waypoints_;
    std::vector<Eigen::Vector3f> trajectory_;
    size_t traj_index_{0};

    // ================= CALLBACKS =================
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_position_ = {
            msg->position[0],
            msg->position[1],
            msg->position[2]
        };

        Eigen::Quaternionf q(
            msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

        Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        current_yaw_ = euler[2];
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        arm_state_ = msg->arming_state;
    }

    // ================= UTIL =================
    bool reached_takeoff_pose()
    {
        Eigen::Vector3f target(0.0f, 0.0f, -4.0f);

        float pos_err = (current_position_ - target).norm();
        float yaw_err = std::atan2(
            std::sin(current_yaw_ - M_PI_2),
            std::cos(current_yaw_ - M_PI_2));

        return (pos_err < POS_TOL) && (std::abs(yaw_err) < YAW_TOL);
    }

    float yaw_from_delta(const Eigen::Vector3f& a,
                         const Eigen::Vector3f& b)
    {
        return std::atan2(b.y() - a.y(), b.x() - a.x());
    }

    // ================= BEZIER =================
    std::vector<Eigen::Vector3f> generate_bezier(
        const Eigen::Vector3f& p0,
        const Eigen::Vector3f& p1,
        const Eigen::Vector3f& p2,
        int N)
    {
        std::vector<Eigen::Vector3f> curve;
        for (int i = 0; i <= N; i++)
        {
            float t = float(i) / N;
            curve.push_back(
                (1 - t) * (1 - t) * p0 +
                2 * (1 - t) * t * p1 +
                t * t * p2
            );
        }
        return curve;
    }

    // ================= PATH (FIXED) =================
   void build_rounded_path()
{
    trajectory_.clear();

    const float D_MAX = 1.0f;
    const int N_LINE  = 200;
    const int N_CURVE = 200;

    // Precompute entry/exit points
    std::vector<Eigen::Vector3f> entries, exits;

    for (size_t i = 1; i < waypoints_.size() - 1; i++)
    {
        Eigen::Vector3f p0 = waypoints_[i - 1];
        Eigen::Vector3f p1 = waypoints_[i];
        Eigen::Vector3f p2 = waypoints_[i + 1];

        float len_in  = (p1 - p0).norm();
        float len_out = (p2 - p1).norm();

        float D_safe = std::min(D_MAX, 0.5f * std::min(len_in, len_out));

        Eigen::Vector3f dir_in  = (p1 - p0).normalized();
        Eigen::Vector3f dir_out = (p2 - p1).normalized();

        entries.push_back(p1 - D_safe * dir_in);
        exits.push_back(p1 + D_safe * dir_out);
    }

    // ===== First straight segment =====
    for (int k = 0; k < N_LINE; k++)
    {
        float t = float(k) / N_LINE;
        trajectory_.push_back(
            waypoints_[0] + t * (entries[0] - waypoints_[0])
        );
    }

    // ===== Middle segments =====
    for (size_t i = 0; i < entries.size(); i++)
    {
        // Corner curve
        auto curve = generate_bezier(
            entries[i],
            waypoints_[i + 1],
            exits[i],
            N_CURVE
        );
        trajectory_.insert(trajectory_.end(), curve.begin(), curve.end());

        // Straight to next entry (if exists)
        if (i + 1 < entries.size())
        {
            for (int k = 0; k < N_LINE; k++)
            {
                float t = float(k) / N_LINE;
                trajectory_.push_back(
                    exits[i] + t * (entries[i + 1] - exits[i])
                );
            }
        }
    }

    // ===== Final straight =====
    for (int k = 0; k < N_LINE; k++)
    {
        float t = float(k) / N_LINE;
        trajectory_.push_back(
            exits.back() + t * (waypoints_.back() - exits.back())
        );
    }

    // RViz visualization
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = now();
    m.ns = "rounded_path";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 0.05;
    m.color.g = 1.0;
    m.color.a = 1.0;

    for (auto &p : trajectory_)
    {
        geometry_msgs::msg::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = p.z();
        m.points.push_back(gp);
    }

    traj_marker_pub_->publish(m);

    RCLCPP_INFO(get_logger(), "Rounded-corner trajectory generated (ORDER FIXED)");
}


    // ================= MAIN LOOP =================
    void cmdloop_callback()
    {
        publish_offboard();

        if (state_ == "INIT")
        {
            publish_sp(0, 0, -1.5, M_PI_2);
            state_ = "OFFBOARD";
        }
        else if (state_ == "OFFBOARD")
        {
            set_offboard();
            arm();
            publish_sp(0, 0, -4.0, M_PI_2);
            state_ = "TAKEOFF";
        }
        else if (state_ == "TAKEOFF")
        {
            publish_sp(0, 0, -4.0, M_PI_2);

            if (arm_state_ ==
                px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
                reached_takeoff_pose())
            {
                build_rounded_path();
                traj_index_ = 0;
                state_ = "FLY";
            }
        }
        else if (state_ == "FLY")
        {
            if (traj_index_ + 1 < trajectory_.size())
            {
                auto& p = trajectory_[traj_index_];
                auto& n = trajectory_[traj_index_ + 1];
                publish_sp(p.x(), p.y(), p.z(), yaw_from_delta(p, n));
                traj_index_++;
            }
            else
            {
                land();
                state_ = "END";
            }
        }
    }

    // ================= PX4 =================
    void publish_offboard()
    {
        px4_msgs::msg::OffboardControlMode m;
        m.timestamp = clock_.now().nanoseconds() / 1000;
        m.position = true;
        offboard_control_mode_pub_->publish(m);
    }

    void publish_sp(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint m;
        m.timestamp = clock_.now().nanoseconds() / 1000;
        m.position = {x, y, z};
        m.yaw = yaw;
        trajectory_setpoint_pub_->publish(m);
    }

    void set_offboard()
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
        px4_msgs::msg::VehicleCommand m;
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoundedCornerTrajectoryController>());
    rclcpp::shutdown();
    return 0;
}
