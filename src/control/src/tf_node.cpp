#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;

class TfNode : public rclcpp::Node
{
public:
    TfNode() : Node("tf_node")
    {
        // Use simulation time
        this->set_parameter(rclcpp::Parameter("use_sim_time", false));

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&TfNode::odom_callback, this, _1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&TfNode::publish_tf, this));

        RCLCPP_INFO(this->get_logger(), "TF node running (world -> base_link)");
    }

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // -------------------------
        // Position: NED → ENU
        // -------------------------
        pos_.x =  msg->position[1];   // East
        pos_.y =  msg->position[0];   // North
        pos_.z = -msg->position[2];   // Up

        // ---------------------------------------------
        // Orientation
        // PX4: body (FRD) w.r.t world (NED)
        // ---------------------------------------------
        tf2::Quaternion q_ned_frd(
            msg->q[1], msg->q[2], msg->q[3], msg->q[0]);

        // NED → ENU
        tf2::Quaternion q_ned_to_enu;
        q_ned_to_enu.setRPY(M_PI, 0.0, M_PI_2);

        // FRD → FLU
        tf2::Quaternion q_frd_to_flu;
        q_frd_to_flu.setRPY(M_PI, 0.0,0.0);

        // Final: base_link (FLU) w.r.t world (ENU)
        q_enu_flu_ = q_ned_to_enu * q_ned_frd * q_frd_to_flu;
        q_enu_flu_.normalize();

        valid_ = true;
    }

    void publish_tf()
    {
        if (!valid_)
            return;

        // Use ROS sim time
        rclcpp::Time now = this->get_clock()->now();

        // Wait until /clock is active
        if (now.nanoseconds() == 0)
            return;

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = "world";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = pos_.x;
        tf.transform.translation.y = pos_.y;
        tf.transform.translation.z = pos_.z;

        tf.transform.rotation.x = q_enu_flu_.x();
        tf.transform.rotation.y = q_enu_flu_.y();
        tf.transform.rotation.z = q_enu_flu_.z();
        tf.transform.rotation.w = q_enu_flu_.w();

        tf_broadcaster_->sendTransform(tf);
    }

    // ROS
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // State
    geometry_msgs::msg::Vector3 pos_;
    tf2::Quaternion q_enu_flu_;
    bool valid_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfNode>());
    rclcpp::shutdown();
    return 0;
}
