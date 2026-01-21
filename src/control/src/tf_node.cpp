#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;

class TfPointCloudTimeBridge : public rclcpp::Node
{
public:
    TfPointCloudTimeBridge()
        : Node("tf_pointcloud_time_bridge")
    {
        sub_odometry_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&TfPointCloudTimeBridge::odometry_callback, this, _1));

        // sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/world/default/model/x500_depth_0/link/realsense/base_link/sensor/realsense_d435/points",
        //     rclcpp::SensorDataQoS(),
        //     std::bind(&TfPointCloudTimeBridge::cloud_callback, this, _1));

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(),
                    "TF–PointCloud Time Bridge running (FRD→FLU fixed)");
    }

private:
    // ================= PX4 ODOM =================
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // ---------- Position: NED → ENU ----------
        latest_base_pose_.position.x = msg->position[1];   // East
        latest_base_pose_.position.y = msg->position[0];   // North
        latest_base_pose_.position.z = -msg->position[2];  // Up

        // ---------- Orientation: FRD → FLU ----------
        // PX4 quaternion: [w, x, y, z] in FRD
        tf2::Quaternion q_frd(
            msg->q[1],  // x
            msg->q[2],  // y
            msg->q[3],  // z
            msg->q[0]   // w
        );

        // Rotate 180° about X axis to convert FRD → FLU
        tf2::Quaternion q_frd_to_flu;
        q_frd_to_flu.setRPY(M_PI, 0.0, M_PI_2);

        tf2::Quaternion q_flu = q_frd_to_flu * q_frd;
        q_flu.normalize();

        latest_base_pose_.orientation = tf2::toMsg(q_flu);
        base_pose_valid_ = true;
    }

    // ================= CLOUD CALLBACK =================
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {
        if (!base_pose_valid_)
            return;

        // ---------- Monotonic TF time clamp ----------
        rclcpp::Time cloud_stamp = cloud->header.stamp;
        rclcpp::Time tf_stamp = cloud_stamp;

        if (last_tf_stamp_.nanoseconds() > 0 &&
            tf_stamp <= last_tf_stamp_)
        {
            tf_stamp = last_tf_stamp_ + rclcpp::Duration(0, 1); // +1 ns
        }
        last_tf_stamp_ = tf_stamp;

        // ---------- world → base_link ----------
        geometry_msgs::msg::TransformStamped tf_world_base;
        tf_world_base.header.stamp = tf_stamp;
        tf_world_base.header.frame_id = "world";
        tf_world_base.child_frame_id = "base_link";
        tf_world_base.transform.translation.x = latest_base_pose_.position.x;
        tf_world_base.transform.translation.y = latest_base_pose_.position.y;
        tf_world_base.transform.translation.z = latest_base_pose_.position.z;
        tf_world_base.transform.rotation = latest_base_pose_.orientation;
        tf_broadcaster_->sendTransform(tf_world_base);

        // ---------- base_link → camera_link ----------
        geometry_msgs::msg::TransformStamped tf_base_cam;
        tf_base_cam.header.stamp = tf_stamp;
        tf_base_cam.header.frame_id = "base_link";
        tf_base_cam.child_frame_id = "camera_link";

        // Camera mounting (ROS FLU)
        tf_base_cam.transform.translation.y = 0.0;
        tf_base_cam.transform.translation.x = 0.166;
        tf_base_cam.transform.translation.z = 0.053;

        tf2::Quaternion q_cam;
        q_cam.setRPY(-M_PI_2, 0, -M_PI_2);
        q_cam.normalize();
        tf_base_cam.transform.rotation = tf2::toMsg(q_cam);
        tf_broadcaster_->sendTransform(tf_base_cam);

        // ---------- camera_link → alias ----------
        geometry_msgs::msg::TransformStamped tf_alias;
        tf_alias.header.stamp = tf_stamp;
        tf_alias.header.frame_id = "camera_link";
        tf_alias.child_frame_id =
            "camera_color_optical_frame";
        tf_alias.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_alias);
    }

    // ================= ROS =================
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::Pose latest_base_pose_;
    bool base_pose_valid_{false};

    rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};
};

// ===================== MAIN =====================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfPointCloudTimeBridge>());
    rclcpp::shutdown();
    return 0;
}
