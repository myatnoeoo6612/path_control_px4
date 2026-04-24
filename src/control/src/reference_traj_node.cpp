// #include <rclcpp/rclcpp.hpp>
// #include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
// #include <geometry_msgs/msg/transform.hpp>
// #include <fstream>
// #include <sstream>

// struct TrajPoint {
//     double t, x, y, z, yaw;
// };

// class ReferenceTrajNode : public rclcpp::Node
// {
// public:
//     ReferenceTrajNode() : Node("reference_traj_node")
//     {
//         pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
//             "/reference_trajectory", 100);

//         load_txt("src/control/config/square.txt");

//         timer_ = create_wall_timer(
//             std::chrono::milliseconds(100),
//             std::bind(&ReferenceTrajNode::publish_traj, this));
//     }

// private:
//     rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::vector<TrajPoint> traj_;

//     void load_txt(const std::string &path)
//     {
//         std::ifstream file(path);
//         std::string line;

//         while (std::getline(file, line))
//         {
//             std::stringstream ss(line);
//             TrajPoint p;
//             ss >> p.t >> p.x >> p.y >> p.z >> p.yaw;
//             traj_.push_back(p);
//         }
//     }

//     void publish_traj()
//     {
//         trajectory_msgs::msg::MultiDOFJointTrajectory msg;
//         msg.header.stamp = now();

//         for (auto &pt : traj_)
//         {
//             trajectory_msgs::msg::MultiDOFJointTrajectoryPoint p;

//             geometry_msgs::msg::Transform t;
//             t.translation.x = pt.x;
//             t.translation.y = pt.y;
//             t.translation.z = pt.z;

//             p.transforms.push_back(t);
//             p.time_from_start = rclcpp::Duration::from_seconds(pt.t);

//             msg.points.push_back(p);
//         }

//         pub_->publish(msg);
//     }
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ReferenceTrajNode>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <fstream>
#include <sstream>

struct TrajPoint {
    double t, x, y, z, yaw;
};

class ReferenceTrajNode : public rclcpp::Node
{
public:
    ReferenceTrajNode() : Node("reference_traj_node")
    {
       //this->declare_parameter("use_sim_time", true);

        traj_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "/reference_trajectory", 100);

        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/reference_path", 100);

        load_txt("src/control/config/square.txt");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ReferenceTrajNode::publish_traj, this));

        RCLCPP_INFO(get_logger(), "Reference trajectory node started");
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<TrajPoint> traj_;

    void load_txt(const std::string &path)
    {
        std::ifstream file(path);
        std::string line;

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            TrajPoint p;
            ss >> p.t >> p.x >> p.y >> p.z >> p.yaw;
            traj_.push_back(p);
        }

        RCLCPP_INFO(get_logger(), "Loaded %ld trajectory points", traj_.size());
    }

    void publish_traj()
    {
        auto now_time = this->get_clock()->now();

        // ===== TRAJECTORY =====
        trajectory_msgs::msg::MultiDOFJointTrajectory traj_msg;
        traj_msg.header.stamp = now_time;
        traj_msg.header.frame_id = "world";

        // ===== PATH (for RViz) =====
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now_time;
        path_msg.header.frame_id = "world";

        for (auto &pt : traj_)
        {
            // ---- trajectory ----
            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint p;

            geometry_msgs::msg::Transform t;
            t.translation.x = pt.x;
            t.translation.y = pt.y;
            t.translation.z = pt.z;

            p.transforms.push_back(t);
            p.time_from_start = rclcpp::Duration::from_seconds(pt.t);

            traj_msg.points.push_back(p);

            // ---- path (RViz) ----
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;

            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.position.z = pt.z;

            path_msg.poses.push_back(pose);
        }

        traj_pub_->publish(traj_msg);
        path_pub_->publish(path_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReferenceTrajNode>());
    rclcpp::shutdown();
    return 0;
}