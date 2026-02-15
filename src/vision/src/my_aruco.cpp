#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class MultiArucoNode : public rclcpp::Node
{
public:
    MultiArucoNode() : Node("multi_aruco_detector")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth_camera/image", 10,
            std::bind(&MultiArucoNode::image_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_poses", 100);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_image", 100);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

        /* ---- Camera intrinsics ---- */
        camera_matrix_ = (cv::Mat_<double>(3,3) <<
            432.49, 0, 320,
            0, 432.49, 240,
            0, 0, 1);

        dist_coeffs_ = cv::Mat::zeros(5,1,CV_64F);

        /* ---- Marker sizes ---- */
        marker_size_map_[0] = 0.30;
        marker_size_map_[1] = 0.10;
        marker_size_map_[2] = 0.10;
        marker_size_map_[3] = 0.10;
        marker_size_map_[4] = 0.10;
        marker_size_map_[5] = 0.10;

        /* ---- Detector parameter tuning (IMPORTANT) ---- */
        detector_params_ = cv::aruco::DetectorParameters::create();

        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 23;
        detector_params_->adaptiveThreshWinSizeStep = 10;

        detector_params_->minMarkerPerimeterRate = 0.01;
        detector_params_->maxMarkerPerimeterRate = 4.0;

        detector_params_->polygonalApproxAccuracyRate = 0.03;

        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = 5;
        detector_params_->cornerRefinementMaxIterations = 30;
        detector_params_->cornerRefinementMinAccuracy = 0.01;
    }

private:

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(
            image,
            dictionary_,
            corners,
            ids,
            detector_params_);

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header = msg->header;

        if(!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(image, corners, ids);

            for(size_t i=0;i<ids.size();i++)
            {
                int id = ids[i];

                double marker_size = 0.1;
                if(marker_size_map_.count(id))
                    marker_size = marker_size_map_[id];

                std::vector<cv::Vec3d> rvecs, tvecs;
                std::vector<std::vector<cv::Point2f>> single_corner;
                single_corner.push_back(corners[i]);

                cv::aruco::estimatePoseSingleMarkers(
                    single_corner,
                    marker_size,
                    camera_matrix_,
                    dist_coeffs_,
                    rvecs,
                    tvecs);

                cv::aruco::drawAxis(image,
                                    camera_matrix_,
                                    dist_coeffs_,
                                    rvecs[0],
                                    tvecs[0],
                                    marker_size*0.5);

                geometry_msgs::msg::Pose pose;

                pose.position.x = tvecs[0][0];
                pose.position.y = tvecs[0][1];
                pose.position.z = tvecs[0][2];

                cv::Mat R;
                cv::Rodrigues(rvecs[0], R);

                tf2::Matrix3x3 tf_R(
                    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));

                tf2::Quaternion q;
                tf_R.getRotation(q);

                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();

                pose_array.poses.push_back(pose);
            }
        }

        pose_pub_->publish(pose_array);

        auto img_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
        image_pub_->publish(*img_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    std::map<int,double> marker_size_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiArucoNode>());
    rclcpp::shutdown();
    return 0;
}
