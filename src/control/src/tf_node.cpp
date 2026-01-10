// #include <memory>
// #include <rclcpp/rclcpp.hpp>

// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>

// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <px4_msgs/msg/vehicle_odometry.hpp>

// using std::placeholders::_1;

// class PX4TFPublisher : public rclcpp::Node
// {
// public:
//     PX4TFPublisher() : Node("px4_tf_publisher")
//     {
//         odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
//             "/fmu/out/vehicle_odometry",
//             rclcpp::SensorDataQoS(),
//             std::bind(&PX4TFPublisher::odom_callback, this, _1));

//         tf_broadcaster_ =
//             std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//         timer_ = create_wall_timer(
//             std::chrono::milliseconds(30),
//             std::bind(&PX4TFPublisher::publish_tf, this));

//         RCLCPP_INFO(get_logger(), "PX4 TF Publisher started");
//     }

// private:
//     // ================= PX4 ODOM =================
//     void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
//     {
//         // Position: NED → ENU
//         pos_.x =  msg->position[1];   // East
//         pos_.y =  msg->position[0];   // North
//         pos_.z = -msg->position[2];   // Up

//         // PX4 quaternion: body(FRD) wrt world(NED)
//         tf2::Quaternion q_ned_frd(
//             msg->q[1], msg->q[2], msg->q[3], msg->q[0]);

//         // NED → ENU
//         tf2::Quaternion q_ned_to_enu;
//         q_ned_to_enu.setRPY(M_PI, 0.0, M_PI_2);

//         // FRD → FLU
//         tf2::Quaternion q_frd_to_flu;
//         q_frd_to_flu.setRPY(M_PI, 0.0, M_PI);

//         q_enu_flu_ = q_ned_to_enu * q_ned_frd * q_frd_to_flu;
//         q_enu_flu_.normalize();

//         valid_ = true;
//     }

//     // ================= TF PUBLISH =================
//     void publish_tf()
//     {
//         if (!valid_) return;

//         rclcpp::Time now = get_clock()->now();

//         // world → base_link
//         geometry_msgs::msg::TransformStamped tf_base;
//         tf_base.header.stamp = now;
//         tf_base.header.frame_id = "world";
//         tf_base.child_frame_id = "base_link";
//         tf_base.transform.translation.x = pos_.x;
//         tf_base.transform.translation.y = pos_.y;
//         tf_base.transform.translation.z = pos_.z;
//         tf_base.transform.rotation = tf2::toMsg(q_enu_flu_);
//         tf_broadcaster_->sendTransform(tf_base);

//         // base_link → camera_link
//         geometry_msgs::msg::TransformStamped tf_cam;
//         tf_cam.header.stamp = now;
//         tf_cam.header.frame_id = "base_link";
//         tf_cam.child_frame_id = "camera_link";
//         tf_cam.transform.translation.x = 0.066;
//         tf_cam.transform.translation.y = 0.0;
//         tf_cam.transform.translation.z = -0.053;

//         tf2::Quaternion q_cam;
//         q_cam.setRPY(0.0, M_PI_2, 0.0);  // downward-facing camera
//         tf_cam.transform.rotation = tf2::toMsg(q_cam);
//         tf_broadcaster_->sendTransform(tf_cam);

//         // camera_link → camera_optical_link  ✅ CRITICAL
//         geometry_msgs::msg::TransformStamped tf_opt;
//         tf_opt.header.stamp = now;
//         tf_opt.header.frame_id = "camera_link";
//         tf_opt.child_frame_id = "camera_optical_link";

//         tf2::Quaternion q_opt;
//         q_opt.setRPY(-M_PI_2, 0.0, -M_PI_2);
//         tf_opt.transform.rotation = tf2::toMsg(q_opt);

//         tf_broadcaster_->sendTransform(tf_opt);
//     }

//     // ================= MEMBERS =================
//     rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
//     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     geometry_msgs::msg::Point pos_;
//     tf2::Quaternion q_enu_flu_;
//     bool valid_{false};
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PX4TFPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;

class PX4BaseLinkTF : public rclcpp::Node
{
public:
    PX4BaseLinkTF()
        : Node("px4_base_link_tf")
    {
        sub_odometry_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&PX4BaseLinkTF::odometry_callback, this, _1));

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(),
                    "PX4 Base Link TF publisher started (world → base_link)");
    }

private:
    void odometry_callback(
        const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // ---------------- Position: NED → ENU ----------------
        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "world";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = msg->position[1];   // East
        tf.transform.translation.y = msg->position[0];   // North
        tf.transform.translation.z = -msg->position[2];  // Up

        // ---------------- Orientation: FRD → FLU ----------------
        // PX4 quaternion is [w, x, y, z] (FRD)
        tf2::Quaternion q_frd(
            msg->q[1],
            msg->q[2],
            msg->q[3],
            msg->q[0]);

        // Convert FRD → FLU
        tf2::Quaternion q_frd_to_flu;
        q_frd_to_flu.setRPY(M_PI, 0.0, M_PI);

        tf2::Quaternion q_flu = q_frd_to_flu * q_frd;
        q_flu.normalize();

        tf.transform.rotation = tf2::toMsg(q_flu);

        tf_broadcaster_->sendTransform(tf);
    }

    // ---------------- ROS ----------------
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4BaseLinkTF>());
    rclcpp::shutdown();
    return 0;
}
