#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <deque>
#include <cmath>

class PX4Visualizer : public rclcpp::Node {
public:
PX4Visualizer() : Node("px4_visualizer") {
  using namespace std::placeholders;

  // Fix: Use BEST_EFFORT to match PX4's default publishing policy
  auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
    "/fmu/out/vehicle_attitude", qos_sub,
    std::bind(&PX4Visualizer::vehicle_attitude_callback, this, _1));

  local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos_sub,
    std::bind(&PX4Visualizer::local_position_callback, this, _1));

  setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos_sub,
    std::bind(&PX4Visualizer::trajectory_setpoint_callback, this, _1));

  // Publishers
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/px4_visualizer/vehicle_pose", 100);
  velocity_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/px4_visualizer/vehicle_velocity", 100);
  vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/px4_visualizer/vehicle_path", 100);
  setpoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/px4_visualizer/setpoint_path", 100);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&PX4Visualizer::timer_callback, this));
}


private:
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Eigen::Vector3f vehicle_position_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f vehicle_velocity_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf vehicle_attitude_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f setpoint_position_ = Eigen::Vector3f::Zero();

  nav_msgs::msg::Path vehicle_path_;
  nav_msgs::msg::Path setpoint_path_;
  size_t trail_size_ = 1000;

  void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    vehicle_attitude_ = Eigen::Quaternionf(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  }

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    vehicle_position_ = Eigen::Vector3f(msg->y, msg->x, -msg->z);
    vehicle_velocity_ = Eigen::Vector3f(msg->vy, msg->vx, -msg->vz);
  }

  void trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
    setpoint_position_ = Eigen::Vector3f(msg->position[1], msg->position[0], -msg->position[2]);
  }

  geometry_msgs::msg::PoseStamped make_pose_msg(const Eigen::Vector3f &pos, const Eigen::Quaternionf &quat) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = pos.x();
    pose.pose.position.y = pos.y();
    pose.pose.position.z = pos.z();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();
    return pose;
  }

  void publish_path(nav_msgs::msg::Path &path, const geometry_msgs::msg::PoseStamped &pose, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pub) {
    path.header = pose.header;
    path.poses.push_back(pose);
    if (path.poses.size() > trail_size_) path.poses.erase(path.poses.begin());
    pub->publish(path);
  }

  void timer_callback() {
    auto pose_msg = make_pose_msg(vehicle_position_, vehicle_attitude_);
    pose_pub_->publish(pose_msg);
    publish_path(vehicle_path_, pose_msg, vehicle_path_pub_);

    auto setpoint_pose = make_pose_msg(setpoint_position_, vehicle_attitude_);
    publish_path(setpoint_path_, setpoint_pose, setpoint_path_pub_);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "velocity_arrow";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point tail, head;
    tail.x = vehicle_position_.x(); tail.y = vehicle_position_.y(); tail.z = vehicle_position_.z();
    head.x = tail.x + 0.3 * vehicle_velocity_.x();
    head.y = tail.y + 0.3 * vehicle_velocity_.y();
    head.z = tail.z + 0.3 * vehicle_velocity_.z();
    marker.points = {tail, head};
    velocity_marker_pub_->publish(marker);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4Visualizer>());
  rclcpp::shutdown();
  return 0;
}
