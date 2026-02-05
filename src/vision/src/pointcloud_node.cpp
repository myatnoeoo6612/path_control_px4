#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

using std::placeholders::_1;

class DepthRGBToPointCloud : public rclcpp::Node
{
public:
    DepthRGBToPointCloud()
        : Node("depth_rgb_to_pointcloud")
    {
        rclcpp::QoS qos(10);
        qos.reliable();
        qos.keep_last(10);

        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            "/world/default/model/x500_depth_0/link/realsense/base_link/sensor/realsense_d435/depth_image",
            qos,
            std::bind(&DepthRGBToPointCloud::depth_callback, this, _1));

        sub_rgb_ = create_subscription<sensor_msgs::msg::Image>(
            "/world/default/model/x500_depth_0/link/realsense/base_link/sensor/realsense_d435/image",
            qos,
            std::bind(&DepthRGBToPointCloud::rgb_callback, this, _1));

        sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/world/default/model/x500_depth_0/link/realsense/base_link/sensor/realsense_d435/camera_info",
            qos,
            std::bind(&DepthRGBToPointCloud::info_callback, this, _1));

        pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points_xyzrgb",
            qos);

        RCLCPP_INFO(get_logger(), "Depth RGB → PointCloud2 node started");
    }

private:
    /* ============================
     * Camera intrinsics
     * ============================ */
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        has_info_ = true;
    }

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rgb_img_ = msg;
    }

    /* ============================
     * Depth → PointCloud
     * ============================ */
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr depth_msg)
    {
        if (!has_info_ || !rgb_img_)
            return;

        cv::Mat depth, rgb;

        try
        {
            depth = cv_bridge::toCvShare(depth_msg)->image;
            rgb = cv_bridge::toCvShare(rgb_img_)->image;
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "cv_bridge conversion failed");
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud->header.frame_id =
            "x500_depth_0/realsense/base_link/realsense_d435";

        cloud->is_dense = false;
        cloud->points.reserve(depth.rows * depth.cols);

        for (int v = 0; v < depth.rows; ++v)
        {
            for (int u = 0; u < depth.cols; ++u)
            {
                float z;

                if (depth.type() == CV_16UC1)
                {
                    z = depth.at<uint16_t>(v, u) * 0.001f;
                }
                else if (depth.type() == CV_32FC1)
                {
                    z = depth.at<float>(v, u);
                }
                else
                {
                    continue;
                }

                if (!std::isfinite(z) || z <= 0.1f || z > 10.0f)
                    continue;

                pcl::PointXYZRGB p;

                p.z = z;
                p.x = (u - cx_) * z / fx_;
                p.y = (v - cy_) * z / fy_;

                const cv::Vec3b &c = rgb.at<cv::Vec3b>(v, u);
                p.r = c[2];
                p.g = c[1];
                p.b = c[0];

                cloud->points.push_back(p);
            }
        }

        // ✅ UNORGANIZED CLOUD (IMPORTANT)
        cloud->width = cloud->points.size();
        cloud->height = 1;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = depth_msg->header.stamp;
        msg.header.frame_id = cloud->header.frame_id;

        pub_cloud_->publish(msg);
    }

    /* ============================
     * ROS interfaces
     * ============================ */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

    sensor_msgs::msg::Image::SharedPtr rgb_img_;

    bool has_info_ = false;
    double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthRGBToPointCloud>());
    rclcpp::shutdown();
    return 0;
}
