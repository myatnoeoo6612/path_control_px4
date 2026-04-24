#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class StaticTFNode : public rclcpp::Node
{
public:
  StaticTFNode() : Node("static_tf")
  {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publish_map_to_world();
    publish_camera_chain();
    publish_base_to_camera();

    RCLCPP_INFO(this->get_logger(), "Static TFs published");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

  void publish_map_to_world()
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "world";

    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    broadcaster_->sendTransform(tf);
  }

  void publish_camera_chain()
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "camera_link";
    tf.child_frame_id = "camera_color_optical_frame";

    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    broadcaster_->sendTransform(tf);
  }

  void publish_base_to_camera()
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "camera_link";

    tf.transform.translation.x = 0.10;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = -0.038;

    tf2::Quaternion q;
    //q.setRPY(0.0, 1.570796, 0.0);  // roll, pitch, yaw
    q.setRPY(0.0, 0.0, 0.0);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    broadcaster_->sendTransform(tf);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTFNode>());
  rclcpp::shutdown();
  return 0;
}
