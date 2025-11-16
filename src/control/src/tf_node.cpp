#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;

class PoseFilterPublisher : public rclcpp::Node
{
public:
    PoseFilterPublisher() : Node("tf_node")
    {
        sub_odometry_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&PoseFilterPublisher::odometry_callback, this, _1));

        base_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_link", 100);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&PoseFilterPublisher::broadcast_transforms, this));
    }

private:
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        geometry_msgs::msg::Pose pose;

        // NED → ENU
        pose.position.x = msg->position[1];   // East
        pose.position.y = msg->position[0];   // North
        pose.position.z = -msg->position[2];  // Up

        // Quaternion NED → ENU
        tf2::Quaternion q_ned(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);

        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, 0);   // If you want -90° rotation, change here

        tf2::Quaternion q_enu = q_rot * q_ned;
        q_enu.normalize();

        pose.orientation.x = q_enu.x();
        pose.orientation.y = q_enu.y();
        pose.orientation.z = q_enu.z();
        pose.orientation.w = q_enu.w();

        latest_base_pose_ = pose;
        base_pose_valid_ = true;
        latest_stamp_ = this->get_clock()->now();
    }

    void broadcast_transforms()
    {
        if (!base_pose_valid_)
            return;

        rclcpp::Time now = this->get_clock()->now();

        // Publish PoseStamped
        publish_pose(base_pub_, latest_base_pose_, now, "world");

        // Publish TF world → base_link
        publish_dynamic_tf("world", "base_link", latest_base_pose_, now);
    }

    // ----------------- UTILITY FUNCTIONS -----------------

    geometry_msgs::msg::Pose rotate_pose_about_z(const geometry_msgs::msg::Pose &pose, double angle_rad)
    {
        tf2::Transform tf, rot;

        tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
        tf.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, angle_rad);
        rot.setRotation(q_rot);

        tf2::Transform result = rot * tf;

        geometry_msgs::msg::Pose new_pose;
        new_pose.position.x = result.getOrigin().x();
        new_pose.position.y = result.getOrigin().y();
        new_pose.position.z = result.getOrigin().z();

        tf2::Quaternion q = result.getRotation();
        new_pose.orientation.x = q.x();
        new_pose.orientation.y = q.y();
        new_pose.orientation.z = q.z();
        new_pose.orientation.w = q.w();

        return new_pose;
    }

    void publish_pose(
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher,
        const geometry_msgs::msg::Pose &pose,
        const rclcpp::Time &stamp,
        const std::string &frame_id)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.pose = pose;
        publisher->publish(msg);
    }

    void publish_dynamic_tf(
        const std::string &parent,
        const std::string &child,
        const geometry_msgs::msg::Pose &pose,
        const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = parent;
        tf.child_frame_id = child;

        tf.transform.translation.x = pose.position.x;
        tf.transform.translation.y = pose.position.y;
        tf.transform.translation.z = pose.position.z;
        tf.transform.rotation = pose.orientation;

        tf_broadcaster_->sendTransform(tf);
    }

    // ---------------- CLASS MEMBERS ----------------

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pub_;

    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::Pose latest_base_pose_;
    rclcpp::Time latest_stamp_;
    bool base_pose_valid_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseFilterPublisher>());
    rclcpp::shutdown();
    return 0;
}
