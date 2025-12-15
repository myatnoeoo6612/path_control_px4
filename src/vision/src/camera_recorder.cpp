#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraRecorderNode : public rclcpp::Node
{
public:
    CameraRecorderNode() : Node("camera_recorder_node")
    {
        // Subscribe to Gazebo depth camera (ROS2 side of the bridge)
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/world/default/model/x500_depth_1/link/realsense/base_link/sensor/realsense_d435/image",
            10,
            std::bind(&CameraRecorderNode::image_callback, this, std::placeholders::_1));

        // Prepare video writer (RGB output)
        writer_.open("/home/myat/recorded_rgb.avi",
                     cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                     30,
                     cv::Size(640, 480));

        if (!writer_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video writer!");
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;

        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed");
            return;
        }

        // Save to video
        writer_.write(frame);

        // Optional: show preview
        cv::imshow("Camera", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    cv::VideoWriter writer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
