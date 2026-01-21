#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarTimeFix : public rclcpp::Node
{
public:
    LidarTimeFix() : Node("lidar_time_fix")
    {
        rclcpp::QoS qos = rclcpp::SensorDataQoS();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/world/default/model/x500_depth_0/link/link/sensor/lidar_2d_v2/scan/points",
            qos,
            std::bind(&LidarTimeFix::cb, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar/points",
            qos);
    }

private:
    void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto out = *msg;
        out.header.stamp = this->get_clock()->now();
        pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTimeFix>());
    rclcpp::shutdown();
    return 0;
}
