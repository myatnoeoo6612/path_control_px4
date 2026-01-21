#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode() : Node("aruco_detect")
  {
    marker_size_ = 0.1;
    target_id_ = 1;

    fx_ = 432.496042035043;
    fy_ = 432.496042035043;
    cx_ = 320.0;
    cy_ = 240.0;

    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);

    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

    image_sub_ = image_transport::create_subscription(this,
                                                      "camera/camera/color/image_raw",
                                                      std::bind(&ArucoDetectorNode::image_callback, this, _1), "raw");

    image_pub_ = image_transport::create_publisher(this, "/aruco/image");

    pose_cam_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose", 10);

    pose_world_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose_world", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "ArUco detector (CORRECT world pose, PX4-safe) started");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (...)
    {
      return;
    }

    cv::Mat image = cv_ptr->image;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids);

    if (ids.empty())
      return;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    for (size_t i = 0; i < ids.size(); i++)
    {
      if (ids[i] != target_id_)
        continue;

      cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.1);

      publish_all(msg->header, tvecs[i]);
    }

    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void publish_all(const std_msgs::msg::Header& header, const cv::Vec3d& tvec)
  {
    // ================= camera → aruco =================
    double yaw = std::atan2(tvec[0], tvec[2]);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    geometry_msgs::msg::PoseStamped cam_pose;
    cam_pose.header = header;
    cam_pose.header.frame_id = "camera_optical_link";
    cam_pose.pose.position.x = tvec[0];
    cam_pose.pose.position.y = tvec[1];
    cam_pose.pose.position.z = tvec[2];
    cam_pose.pose.orientation = tf2::toMsg(q);

    pose_cam_pub_->publish(cam_pose);

    geometry_msgs::msg::TransformStamped tf_cam;
    tf_cam.header = header;
    tf_cam.header.frame_id = "camera_optical_link";
    tf_cam.child_frame_id = "aruco_1";
    tf_cam.transform.translation.x = tvec[0];
    tf_cam.transform.translation.y = tvec[1];
    tf_cam.transform.translation.z = tvec[2];
    tf_cam.transform.rotation = cam_pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_cam);

    // try
    // {
    //     // ================= world(NED) -> base_link(FRD) =================
    //     auto tf_wb_frd_msg = tf_buffer_->lookupTransform(
    //         "world", "base_link", tf2::TimePointZero);

    //     tf2::Transform T_wb_frd;
    //     tf2::fromMsg(tf_wb_frd_msg.transform, T_wb_frd);

    //     // ================= NED -> ENU =================
    //     // PX4 world: X=N, Y=E, Z=D
    //     // ROS  world: X=E, Y=N, Z=U
    //     tf2::Quaternion q_ned_to_enu;
    //     q_ned_to_enu.setRPY(M_PI, 0.0, M_PI_2); // Rx(180) then Rz(90)

    //     tf2::Transform T_ned_to_enu;
    //     T_ned_to_enu.setIdentity();
    //     T_ned_to_enu.setRotation(q_ned_to_enu);

    //     // world(ENU) -> base(FRD)
    //     tf2::Transform T_wb_ned = T_wb_frd;
    //     tf2::Transform T_wb_enu_frd = T_ned_to_enu * T_wb_ned;

    //     // ================= FRD -> FLU =================
    //     tf2::Quaternion q_frd_to_flu;
    //     q_frd_to_flu.setRPY(M_PI, 0.0, M_PI_2); // FRD -> FLU

    //     tf2::Transform T_frd_to_flu;
    //     T_frd_to_flu.setIdentity();
    //     T_frd_to_flu.setRotation(q_frd_to_flu);

    //     // world(ENU) -> base(FLU)
    //     tf2::Transform T_wb = T_wb_enu_frd * T_frd_to_flu;

    //     // ================= base_link -> camera_link =================
    //     auto tf_bc_msg = tf_buffer_->lookupTransform(
    //         "base_link", "camera_link", tf2::TimePointZero);

    //     tf2::Transform T_bc;
    //     tf2::fromMsg(tf_bc_msg.transform, T_bc);

    //     // ================= camera_link -> aruco =================
    //     tf2::Transform T_ca;
    //     T_ca.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));
    //     T_ca.setRotation(q);

    //     // ================= Compose final =================
    //     tf2::Transform T_wa = T_wb * T_bc * T_ca;

    //     // ================= Publish world pose =================
    //     geometry_msgs::msg::PoseStamped world_pose;
    //     world_pose.header.stamp = get_clock()->now();
    //     world_pose.header.frame_id = "world";

    //     geometry_msgs::msg::Transform tf_msg = tf2::toMsg(T_wa);
    //     world_pose.pose.position.x = tf_msg.translation.x;
    //     world_pose.pose.position.y = tf_msg.translation.y;
    //     world_pose.pose.position.z = tf_msg.translation.z;
    //     world_pose.pose.orientation = tf_msg.rotation;

    //     pose_world_pub_->publish(world_pose);
    // }
    // catch (tf2::TransformException &e)
    // {
    //     RCLCPP_WARN_THROTTLE(
    //         get_logger(), *get_clock(), 2000,
    //         "Waiting for TF: world->base_link->camera_link");
    // }
    try
    {
      auto tf_wb = tf_buffer_->lookupTransform("world", "base_link", tf2::TimePointZero);

      auto tf_bo = tf_buffer_->lookupTransform("base_link", "camera_optical_link", tf2::TimePointZero);

      tf2::Transform T_wb, T_bo, T_oa;
      tf2::fromMsg(tf_wb.transform, T_wb);
      tf2::fromMsg(tf_bo.transform, T_bo);

      // OpenCV pose (already optical frame)
      T_oa.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));
      T_oa.setRotation(q);

      tf2::Transform T_wa = T_wb * T_bo * T_oa;

      geometry_msgs::msg::PoseStamped world_pose;
      world_pose.header.stamp = get_clock()->now();
      world_pose.header.frame_id = "world";

      world_pose.pose.position.x = T_wa.getOrigin().x();
      world_pose.pose.position.y = T_wa.getOrigin().y();
      world_pose.pose.position.z = T_wa.getOrigin().z();
      world_pose.pose.orientation = tf2::toMsg(T_wa.getRotation());

      pose_world_pub_->publish(world_pose);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for TF chain");
    }
  }

  // ================= members =================
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cam_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_world_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Mat camera_matrix_, dist_coeffs_;

  double fx_, fy_, cx_, cy_;
  double marker_size_;
  int target_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
