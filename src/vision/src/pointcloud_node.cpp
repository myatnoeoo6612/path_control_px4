#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using std::placeholders::_1;

class PointCloudVoxelFilter : public rclcpp::Node
{
public:
    PointCloudVoxelFilter()
        : Node("pointcloud_voxel")
    {
        voxel_ = this->declare_parameter<double>("voxel", 0.01);

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/world/default/model/x500_depth_0/link/realsense/base_link/sensor/realsense_d435/points",
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloudVoxelFilter::cloudCallback, this, _1));

        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points_voxel",
            rclcpp::QoS(10).reliable());

        RCLCPP_INFO(this->get_logger(),
                    "PointCloud Voxel Filter (FORCE RVIZ MODE)");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
            return;

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_, voxel_, voxel_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        vg.filter(*out_cloud);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Voxel points: %zu", out_cloud->size());

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*out_cloud, out);

        // 🔥 ABSOLUTE FIX 🔥
        out.header.stamp = this->get_clock()->now();
        out.header.frame_id = "world";   // ⬅ FORCE RViz to render

        pub_cloud_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    double voxel_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudVoxelFilter>());
    rclcpp::shutdown();
    return 0;
}
