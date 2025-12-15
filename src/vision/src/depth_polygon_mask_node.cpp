#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <algorithm>

class DepthPolygonMaskNode : public rclcpp::Node {
public:
    DepthPolygonMaskNode() : Node("depth_polygon_mask_node"), camera_ready_(false) {
        using std::placeholders::_1;

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/world/default/model/x500_depth_1/link/realsense/base_link/sensor/realsense_d435/depth_image", 100,
            std::bind(&DepthPolygonMaskNode::depthCallback, this, _1));

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/world/default/model/x500_depth_1/link/realsense/base_link/sensor/realsense_d435/camera_info", 10,
            std::bind(&DepthPolygonMaskNode::camInfoCallback, this, _1));

        polygon_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/segmented_region", 100,
            std::bind(&DepthPolygonMaskNode::polygonCallback, this, _1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/pose_2d", 10);
        masked_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/masked_depth_image", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr masked_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pose_pub_;

    image_geometry::PinholeCameraModel cam_model_;
    std::vector<cv::Point> polygon_;
    bool camera_ready_;

    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!camera_ready_) {
            cam_model_.fromCameraInfo(*msg);
            camera_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera model loaded.");
        }
    }

    void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        polygon_.clear();
        for (const auto& pt : msg->polygon.points) {
            polygon_.emplace_back(static_cast<int>(std::round(pt.x)), static_cast<int>(std::round(pt.y)));
        }
        RCLCPP_INFO(this->get_logger(), "Received polygon with %zu points.", polygon_.size());
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!camera_ready_ || polygon_.size() < 3) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        auto depth_image = cv_ptr->image;
        int height = depth_image.rows;
        int width = depth_image.cols;

        std::vector<cv::Point> clipped_polygon = polygon_;
        for (auto& pt : clipped_polygon) {
            pt.x = std::clamp(pt.x, 0, width - 1);
            pt.y = std::clamp(pt.y, 0, height - 1);
        }

        cv::Mat mask = cv::Mat::zeros(depth_image.size(), CV_8UC1);
        cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{clipped_polygon}, cv::Scalar(255));

        cv::Mat depth_inside;
        depth_image.copyTo(depth_inside, mask);
        cv::Mat valid_mask = (depth_inside > 0);
        if (cv::countNonZero(valid_mask) == 0) {
            RCLCPP_WARN(this->get_logger(), "No valid depth in polygon.");
            return;
        }

        double min_depth;
        cv::minMaxLoc(depth_inside, &min_depth, nullptr, nullptr, nullptr, valid_mask);
        const double depth_threshold = 100.0;

        cv::Mat nearest_mask = (cv::abs(depth_image - min_depth) < depth_threshold) & (mask == 255);
        cv::Mat masked_depth = cv::Mat::zeros(depth_image.size(), depth_image.type());
        depth_image.copyTo(masked_depth, nearest_mask);

        std::vector<cv::Point> indices;
        cv::findNonZero(nearest_mask, indices);
        if (indices.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "Not enough valid points for centroid.");
            return;
        }

        std::vector<float> xs, ys, zs;
        for (const auto& pt : indices) {
            float z = depth_image.at<uint16_t>(pt) / 1000.0f;
            float x = (pt.x - cam_model_.cx()) * z / cam_model_.fx();
            float y = (pt.y - cam_model_.cy()) * z / cam_model_.fy();
            xs.push_back(x);
            ys.push_back(y);
            zs.push_back(z);
        }

        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header = msg->header;
        point_msg.point.x = std::accumulate(xs.begin(), xs.end(), 0.0f) / xs.size();
        point_msg.point.y = std::accumulate(ys.begin(), ys.end(), 0.0f) / ys.size();
        point_msg.point.z = std::accumulate(zs.begin(), zs.end(), 0.0f) / zs.size();
        pose_pub_->publish(point_msg);

        sensor_msgs::msg::Image::SharedPtr masked_msg = cv_bridge::CvImage(msg->header, msg->encoding, masked_depth).toImageMsg();
        masked_pub_->publish(*masked_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthPolygonMaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}