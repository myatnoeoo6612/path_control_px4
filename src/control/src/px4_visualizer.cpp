#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

using namespace std::chrono_literals;

class PX4Visualizer : public rclcpp::Node
{
public:
    PX4Visualizer() : Node("px4_visualizer")
    {
        auto qos = rclcpp::QoS(10).best_effort();

        // ================= WORLD FRAME =================
        // ROS ENU → PX4 NED
        Eigen::Matrix3f R_enu_ned;
        R_enu_ned << 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1;
        q_enu_to_ned_ = Eigen::Quaternionf(R_enu_ned);

        // ================= BODY FRAME =================
        // PX4 FRD → ROS FLU (180° about X)
        q_frd_to_flu_ = Eigen::Quaternionf(
            Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
        );

        vehicle_attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&PX4Visualizer::attitudeCallback, this, std::placeholders::_1));

        local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&PX4Visualizer::positionCallback, this, std::placeholders::_1));

        setpoint_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos,
            std::bind(&PX4Visualizer::setpointCallback, this, std::placeholders::_1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/px4_visualizer/vehicle_pose", 100);

        vehicle_path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/px4_visualizer/vehicle_path", 100);

        setpoint_path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/px4_visualizer/setpoint_path", 100);

        velocity_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/px4_visualizer/velocity_arrow", 100);

        timer_ = create_wall_timer(
            50ms, std::bind(&PX4Visualizer::timerCallback, this));
    }

private:
    // ================= ROS =================

    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_marker_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ================= STATE =================

    Eigen::Vector3f position_world_{0, 0, 0};     // ENU
    Eigen::Vector3f velocity_world_{0, 0, 0};     // ENU

    Eigen::Quaternionf q_world_body_{1, 0, 0, 0}; // ROS ENU → FLU
    Eigen::Quaternionf q_enu_to_ned_;
    Eigen::Quaternionf q_frd_to_flu_;

    Eigen::Vector3f setpoint_world_{0, 0, 0};

    nav_msgs::msg::Path vehicle_path_;
    nav_msgs::msg::Path setpoint_path_;

    constexpr static size_t TRAIL_SIZE = 500;

    // ================= CALLBACKS =================

    void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        // PX4 quaternion: NED → FRD
        Eigen::Quaternionf q_px4(
            msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

        // ROS quaternion: ENU → FLU
        q_world_body_ = q_enu_to_ned_ * q_px4 * q_frd_to_flu_;
        q_world_body_.normalize();
    }

    void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        // PX4 NED → ROS ENU
        position_world_ = Eigen::Vector3f(msg->y, msg->x, -msg->z);
        velocity_world_ = Eigen::Vector3f(msg->vy, msg->vx, -msg->vz);
    }

    void setpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
    {
        setpoint_world_ = Eigen::Vector3f(
            msg->position[1], msg->position[0], -msg->position[2]);
    }

    // ================= HELPERS =================

    geometry_msgs::msg::PoseStamped makePose(
        const Eigen::Vector3f &pos,
        const Eigen::Quaternionf &q)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "world";
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = pos.z();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        return pose;
    }

    Eigen::Vector3f worldToBody(
        const Eigen::Vector3f &v_world,
        const Eigen::Quaternionf &q_world_body)
    {
        // ENU → FLU
        return q_world_body.inverse() * v_world;
    }

    void pushPath(nav_msgs::msg::Path &path,
                  const geometry_msgs::msg::PoseStamped &pose,
                  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pub)
    {
        path.header = pose.header;
        path.poses.push_back(pose);
        if (path.poses.size() > TRAIL_SIZE)
            path.poses.erase(path.poses.begin());
        pub->publish(path);
    }

    // ================= TIMER =================

    void timerCallback()
    {
        // Pose
        auto pose_msg = makePose(position_world_, q_world_body_);
        pose_pub_->publish(pose_msg);
        pushPath(vehicle_path_, pose_msg, vehicle_path_pub_);

        // Setpoint
        auto sp_pose = makePose(setpoint_world_, q_world_body_);
        pushPath(setpoint_path_, sp_pose, setpoint_path_pub_);

        // -------- Velocity Arrow (BODY FRAME) --------
        Eigen::Vector3f vel_body = worldToBody(
            velocity_world_, q_world_body_);

        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = now();
        arrow.header.frame_id = "base_link";   // FLU
        arrow.ns = "velocity";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        arrow.scale.x = 0.1;
        arrow.scale.y = 0.2;
        arrow.scale.z = 0.2;

        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;

        geometry_msgs::msg::Point p0, p1;
        p0.x = 0.0;
        p0.y = 0.0;
        p0.z = 0.0;

        // X = forward (RED)
        p1.x = vel_body.x();
        p1.y = vel_body.y();
        p1.z = vel_body.z();

        arrow.points = {p0, p1};
        velocity_marker_pub_->publish(arrow);
    }
};

// ================= MAIN =================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4Visualizer>());
    rclcpp::shutdown();
    return 0;
}
