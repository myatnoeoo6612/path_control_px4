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
#include <tf2/LinearMath/Matrix3x3.h>
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
    // ---- Marker + camera params ----
    marker_size_ = 0.1;
    target_id_ = 1;

    fx_ = 432.496042035043;
    fy_ = 432.496042035043;
    cx_ = 320.0;
    cy_ = 240.0;

    camera_matrix_ = (cv::Mat_<double>(3,3) <<
      fx_, 0, cx_,
      0, fy_, cy_,
      0, 0, 1);

    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

    dictionary_ =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

    // ---- ROS interfaces ----
    image_sub_ = image_transport::create_subscription(
      this, "/depth_camera/image",
      std::bind(&ArucoDetectorNode::image_callback, this, _1), "raw");

    image_pub_ =
      image_transport::create_publisher(this, "/aruco/image");

    pose_cam_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose", 10);

    pose_world_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose_world", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(get_clock());

    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(),
      "ArUco detector started (camera_link, yaw-only, stabilized + yaw error)");
  }

private:
  // =====================================================
  // Image callback
  // =====================================================
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
    cv::aruco::estimatePoseSingleMarkers(
      corners, marker_size_,
      camera_matrix_, dist_coeffs_,
      rvecs, tvecs);

    for (size_t i = 0; i < ids.size(); i++)
    {
      if (ids[i] != target_id_)
        continue;

      cv::aruco::drawAxis(
        image, camera_matrix_, dist_coeffs_,
        rvecs[i], tvecs[i], 0.1);

      publish_all(msg->header, tvecs[i], rvecs[i]);
    }

    image_pub_.publish(cv_ptr->toImageMsg());
  }

  // =====================================================
  // Main pose logic
  // =====================================================
  void publish_all(const std_msgs::msg::Header& header,
                   const cv::Vec3d& tvec,
                   const cv::Vec3d& rvec)
  {
    // ---------- rvec -> quaternion ----------
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    tf2::Matrix3x3 R_ocv(
      R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
      R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
      R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
    );

    tf2::Quaternion q_ocv;
    R_ocv.getRotation(q_ocv);

    // ---------- OpenCV -> camera_link ----------
    tf2::Quaternion q_fix;
    q_fix.setRPY(M_PI/2.0, 0.0, -M_PI/2.0);

    tf2::Quaternion q_tmp = q_fix * q_ocv;

    // ---------- Flip marker Z upward ----------
    tf2::Quaternion q_flip;
    q_flip.setRPY(M_PI, 0.0, 0.0);
    q_tmp = q_tmp * q_flip;

    // ---------- Extract & filter yaw ----------
    tf2::Matrix3x3 m(q_tmp);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (!yaw_initialized_)
    {
      filtered_yaw_ = yaw;
      yaw_initialized_ = true;
    }
    else
    {
      double dy = normalize_angle(yaw - filtered_yaw_);
      filtered_yaw_ =
        normalize_angle(filtered_yaw_ + yaw_alpha_ * dy);
    }

    tf2::Quaternion q_final;
    q_final.setRPY(0.0, 0.0, filtered_yaw_);

    // ---------- Translation OpenCV -> camera_link ----------
    tf2::Vector3 t_cam(
      tvec[2],    // X forward
     -tvec[0],    // Y left
     -tvec[1]     // Z up
    );

    // Lock to ground (marker on floor)
    t_cam.setZ(0.0);

    // ---------- Publish camera pose ----------
    geometry_msgs::msg::PoseStamped cam_pose;
    cam_pose.header = header;
    cam_pose.header.frame_id = "camera_link";
    cam_pose.pose.position.x = t_cam.x();
    cam_pose.pose.position.y = t_cam.y();
    cam_pose.pose.position.z = t_cam.z();
    cam_pose.pose.orientation = tf2::toMsg(q_final);
    pose_cam_pub_->publish(cam_pose);

    // ---------- TF: camera_link -> aruco_1 ----------
    geometry_msgs::msg::TransformStamped tf_cam;
    tf_cam.header = header;
    tf_cam.header.frame_id = "camera_link";
    tf_cam.child_frame_id = "aruco_1";
    tf_cam.transform.translation.x = t_cam.x();
    tf_cam.transform.translation.y = t_cam.y();
    tf_cam.transform.translation.z = t_cam.z();
    tf_cam.transform.rotation = tf2::toMsg(q_final);
    tf_broadcaster_->sendTransform(tf_cam);

    // ---------- World pose + yaw error ----------
    try
    {
      auto tf_wb =
        tf_buffer_->lookupTransform("world", "base_link",
                                    tf2::TimePointZero);
      auto tf_bc =
        tf_buffer_->lookupTransform("base_link", "camera_link",
                                    tf2::TimePointZero);

      tf2::Transform T_wb, T_bc, T_ca;
      tf2::fromMsg(tf_wb.transform, T_wb);
      tf2::fromMsg(tf_bc.transform, T_bc);

      T_ca.setOrigin(t_cam);
      T_ca.setRotation(q_final);

      tf2::Transform T_wa = T_wb * T_bc * T_ca;

      geometry_msgs::msg::PoseStamped world_pose;
      world_pose.header.stamp = now();
      world_pose.header.frame_id = "world";
      world_pose.pose.position.x = T_wa.getOrigin().x();
      world_pose.pose.position.y = T_wa.getOrigin().y();
      world_pose.pose.position.z = T_wa.getOrigin().z();
      world_pose.pose.orientation =
        tf2::toMsg(T_wa.getRotation());

      pose_world_pub_->publish(world_pose);

      // ===== Compute yaw difference =====
      tf2::Matrix3x3 R_wb(T_wb.getRotation());
      double br, bp, base_yaw;
      R_wb.getRPY(br, bp, base_yaw);

      tf2::Matrix3x3 R_wa(T_wa.getRotation());
      double ar, ap, aruco_yaw;
      R_wa.getRPY(ar, ap, aruco_yaw);

      double yaw_error =
        normalize_angle(aruco_yaw - base_yaw);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Yaw error (aruco - base): %.2f deg",
        rad2deg(yaw_error)
      );
    }
    catch (tf2::TransformException&)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF chain");
    }
  }

  // =====================================================
  // Helpers
  // =====================================================
  double normalize_angle(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  double rad2deg(double r)
  {
    return r * 180.0 / M_PI;
  }

  // =====================================================
  // Members
  // =====================================================
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

  // ---- Stabilization ----
  double filtered_yaw_ = 0.0;
  bool yaw_initialized_ = false;
  double yaw_alpha_ = 0.12;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
