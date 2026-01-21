#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <algorithm>
#include <array>

class LidarToObstacleDistance : public rclcpp::Node
{
public:
    LidarToObstacleDistance()
        : Node("obstacle_node")
    {
        // ---------- PARAMETERS ----------
        sector_count_ = 72;              // PX4 native resolution
        min_range_    = 0.3f;            // meters
        max_range_    = 5.0f;            // meters
        sector_width_ = 2.0f * M_PI / sector_count_;  // 5 degrees

        // ---------- SUBSCRIBER ----------
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarToObstacleDistance::cloudCallback, this, std::placeholders::_1));

        // ---------- PUBLISHER ----------
        pub_obstacle_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>(
            "/fmu/in/obstacle_distance",
            rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(),
                    "PX4 ObstacleDistance node started (72 sectors, 5 deg)");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // ---------- INIT SECTOR DISTANCES ----------
        std::array<float, 72> sector_dist;
        sector_dist.fill(max_range_);

        // ---------- SECTORIZATION ----------
        for (const auto &p : cloud.points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y))
                continue;

            // PX4 BODY_FRD: X forward, Y right
            float x = p.x;
            float y = p.y;

            float distance = std::hypot(x, y);
            if (distance < min_range_ || distance > max_range_)
                continue;

            float angle = std::atan2(y, x);   // [-pi, pi]

            int sector = static_cast<int>(
                std::floor((angle + M_PI) / sector_width_));

            sector = std::clamp(sector, 0, sector_count_ - 1);

            sector_dist[sector] =
                std::min(sector_dist[sector], distance);
        }

        // ---------- BUILD PX4 MESSAGE ----------
        px4_msgs::msg::ObstacleDistance out;

        out.timestamp =
            this->get_clock()->now().nanoseconds() / 1000; // µs

        out.frame =
            px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD;

        out.increment = 360.0f / sector_count_;   // 5 degrees
        out.min_distance = static_cast<uint16_t>(min_range_ * 100.0f);
        out.max_distance = static_cast<uint16_t>(max_range_ * 100.0f);

        // PX4 expects EXACTLY 72 values
        for (int i = 0; i < 72; i++)
        {
            out.distances[i] =
                static_cast<uint16_t>(sector_dist[i] * 100.0f);
        }

        pub_obstacle_->publish(out);
    }

    // ---------- ROS ----------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr pub_obstacle_;

    // ---------- CONFIG ----------
    int   sector_count_;
    float sector_width_;
    float min_range_;
    float max_range_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarToObstacleDistance>());
    rclcpp::shutdown();
    return 0;
}
