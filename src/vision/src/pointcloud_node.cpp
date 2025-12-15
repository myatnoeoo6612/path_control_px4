#include <memory>
#include <vector>
#include <cmath>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "cv_bridge/cv_bridge.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

using namespace std::chrono_literals;

class DepthToPointCloudNode : public rclcpp::Node
{
public:
    DepthToPointCloudNode()
        : Node("depth_to_pointcloud_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        fx_ = 432.496042035043;
        fy_ = 432.496042035043;
        cx_ = 320.0;
        cy_ = 240.0;
        ema_alpha_ = 0.8;

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/masked_depth_image", 100,
            std::bind(&DepthToPointCloudNode::depth_callback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/my_pointcloud", 50);
        centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/centroid_point", 50);
        world_pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/centroid_world_pose", 50);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/object_yaw", 50);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    double fx_, fy_, cx_, cy_, ema_alpha_;
    Eigen::Vector3d filtered_centroid_;
    bool centroid_initialized_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr world_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ------------------------
    // Correct TF2 Transform
    // ------------------------
    Eigen::Vector3d transform_point(const Eigen::Vector3d &pt,
                                    const geometry_msgs::msg::TransformStamped &tf_msg)
    {
        tf2::Transform tf;
        tf2::fromMsg(tf_msg.transform, tf);
        tf2::Vector3 v(pt.x(), pt.y(), pt.z());
        tf2::Vector3 out = tf * v;
        return {out.x(), out.y(), out.z()};
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 1. Convert depth image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_image = cv_ptr->image.clone();
        int height = depth_image.rows;
        int width = depth_image.cols;

        // 2. Build pointcloud from depth
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->reserve(height * width);

        for (int v = 0; v < height; v++)
        {
            for (int u = 0; u < width; u++)
            {
                float z = depth_image.at<float>(v, u);
                if (!std::isfinite(z) || z <= 0.05f)
                    continue;

                float x = (u - cx_) * z / fx_;
                float y = (v - cy_) * z / fy_;

                cloud->points.emplace_back(x, -z, y);
            }
        }

        if (cloud->points.size() < 20)
        {
            RCLCPP_WARN(this->get_logger(), "Too few depth points.");
            return;
        }

        // 3. Filter cloud
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f);
        voxel.filter(*cloud);

        if (cloud->size() < 10)
        {
            RCLCPP_WARN(this->get_logger(), "Too few points after filtering.");
            return;
        }

        // 4. PCA for centroid + principal axis
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud);
        Eigen::Vector4f centroid4f = pca.getMean();
        Eigen::Vector3d centroid(centroid4f[0], centroid4f[1], centroid4f[2]);

        // EMA smoothing
        if (!centroid_initialized_)
        {
            filtered_centroid_ = centroid;
            centroid_initialized_ = true;
        }
        else
        {
            filtered_centroid_ = ema_alpha_ * centroid + (1.0 - ema_alpha_) * filtered_centroid_;
        }

        Eigen::Vector3d stable_centroid = filtered_centroid_;

        geometry_msgs::msg::Point centroid_msg;
        centroid_msg.x = stable_centroid.x();
        centroid_msg.y = stable_centroid.y();
        centroid_msg.z = stable_centroid.z();
        centroid_pub_->publish(centroid_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Stable Centroid (camera_link): %.2f %.2f %.2f",
                    stable_centroid.x(), stable_centroid.y(), stable_centroid.z());

        // ---------------------------------------
        // 5. Transform to base_link and world
        // Use latest transform (fixes extrapolation)
        // ---------------------------------------
        try
        {
            auto cam_to_base =
                tf_buffer_.lookupTransform("drone_1/base_link",
                                           "drone_1/camera_link",
                                           tf2::TimePointZero);

            auto base_to_world =
                tf_buffer_.lookupTransform("world",
                                           "drone_1/base_link",
                                           tf2::TimePointZero);

            // camera → base
            Eigen::Vector3d pt_base = transform_point(stable_centroid, cam_to_base);

            // base → world
            Eigen::Vector3d pt_world = transform_point(pt_base, base_to_world);

            geometry_msgs::msg::Point world_msg;
            world_msg.x = pt_world.x();
            world_msg.y = pt_world.y();
            world_msg.z = pt_world.z();
            world_pose_pub_->publish(world_msg);

            RCLCPP_INFO(this->get_logger(),
                        "World Centroid: %.2f %.2f %.2f",
                        pt_world.x(), pt_world.y(), pt_world.z());
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        }

        // ---------------------------------------
        // 6. Publish pointcloud
        // ---------------------------------------
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header = msg->header;
        cloud_pub_->publish(cloud_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
