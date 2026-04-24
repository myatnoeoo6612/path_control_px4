// #include <rclcpp/rclcpp.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/vehicle_odometry.hpp>

// #include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

// class TrajectoryController : public rclcpp::Node
// {
// public:
//   TrajectoryController() : Node("trajectory_controller")
//   {
//     traj_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
//         "/reference_trajectory", 10, std::bind(&TrajectoryController::traj_callback, this, std::placeholders::_1));

//     odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
//         "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
//         std::bind(&TrajectoryController::odom_callback, this, std::placeholders::_1));

//     sp_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

//     offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

//     cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

//     timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&TrajectoryController::control_loop, this));

//     RCLCPP_INFO(get_logger(), "Trajectory Controller Started");
//   }

// private:
//   enum State
//   {
//     IDLE,
//     TAKEOFF,
//     TRACK,
//     LAND,
//     DONE
//   };
//   State state_ = IDLE;

//   trajectory_msgs::msg::MultiDOFJointTrajectory traj_;
//   rclcpp::Time traj_start_;
//   bool traj_received_ = false;

//   float current_z_ = 0.0f;

//   // pubs/subs
//   rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_sub_;
//   rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

//   rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
//   rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
//   rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

//   rclcpp::TimerBase::SharedPtr timer_;

//   // ===== CALLBACKS =====
//   void traj_callback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
//   {
//     traj_ = *msg;
//     traj_received_ = true;
//     RCLCPP_INFO(get_logger(), "Trajectory received");
//   }

//   void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
//   {
//     current_z_ = msg->position[2];  // NED frame
//   }

//   // ===== COMMANDS =====
//   void arm()
//   {
//     send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
//   }

//   void set_offboard()
//   {
//     send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
//   }

//   void land()
//   {
//     send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
//   }

//   void send_command(uint16_t cmd, float p1 = 0, float p2 = 0)
//   {
//     px4_msgs::msg::VehicleCommand msg{};
//     msg.command = cmd;
//     msg.param1 = p1;
//     msg.param2 = p2;
//     msg.target_system = 1;
//     msg.target_component = 1;
//     msg.source_system = 1;
//     msg.source_component = 1;
//     msg.from_external = true;
//     msg.timestamp = now().nanoseconds() / 1000;
//     cmd_pub_->publish(msg);
//   }

//   float normalize_yaw(float yaw)
//   {
//     while (yaw > M_PI)
//       yaw -= 2 * M_PI;
//     while (yaw < -M_PI)
//       yaw += 2 * M_PI;
//     return yaw;
//   }

//   // ===== CONTROL LOOP =====
//   void control_loop()
//   {
//     publish_offboard();

//     static int counter = 0;

//     // ===== IDLE → TAKEOFF =====
//     if (state_ == IDLE)
//     {
//       if (counter++ > 20)
//       {
//         set_offboard();
//         arm();
//         state_ = TAKEOFF;
//         RCLCPP_INFO(get_logger(), "Switch to TAKEOFF");
//       }
//       return;
//     }

//     // ===== TAKEOFF =====
//     if (state_ == TAKEOFF)
//     {
//       publish_position(0, 0, -3);

//       if (current_z_ < -2.8)
//       {
//         traj_start_ = now();
//         state_ = TRACK;
//         RCLCPP_INFO(get_logger(), "Start trajectory");
//       }
//       return;
//     }

//     // ===== TRACK =====
//     if (state_ == TRACK)
//     {
//       if (!traj_received_ || traj_.points.empty())
//         return;

//       double t = (now() - traj_start_).seconds();

//       size_t idx = traj_.points.size() - 1;
//       for (size_t i = 0; i < traj_.points.size(); i++)
//       {
//         double t_pt = traj_.points[i].time_from_start.sec + traj_.points[i].time_from_start.nanosec * 1e-9;

//         if (t_pt >= t)
//         {
//           idx = i;
//           break;
//         }
//       }

//       auto& pt = traj_.points[idx];
//       auto tr = pt.transforms[0];

//       size_t next_idx = std::min(idx + 1, traj_.points.size() - 1);
//       auto tr_next = traj_.points[next_idx].transforms[0];

//       float dx = tr_next.translation.x - tr.translation.x;
//       float dy = tr_next.translation.y - tr.translation.y;

//       //   float yaw = atan2f(dy, dx);
//       //   yaw = normalize_yaw(yaw);
//       //   float yaw = 0.0f;
//       //   if (fabs(vx) > 1e-4 || fabs(vy) > 1e-4)
//       //   {
//       //     float yaw_enu = atan2f(vy, vx);

//       //     // 🔥 FIX: convert ENU → NED
//       //     yaw = yaw_enu - M_PI_2;

//       //     yaw = normalize_yaw(yaw);
//       //   }
//       float yaw = 0.0f;
//       if (fabs(dx) > 1e-4 || fabs(dy) > 1e-4)
//       {
//         float yaw_enu = atan2f(dy, dx);
//         yaw = yaw_enu - M_PI_2;
//         yaw = normalize_yaw(yaw);
//       }
//       px4_msgs::msg::TrajectorySetpoint sp{};
//       sp.timestamp = now().nanoseconds() / 1000;

//       //sp.position = { (float)tr.translation.x, (float)tr.translation.y, (float)(-tr.translation.z) };
//       sp.position = {
//           (float)tr.translation.y,      // 🔥 swap
//           (float)tr.translation.x,      // 🔥 swap
//           (float)(-tr.translation.z)
//       };
//       sp.velocity = { dy / 0.1f, dx / 0.1f, 0.0f };
//       sp.yaw = yaw;

//       sp_pub_->publish(sp);

//       // ===== FINISH → LAND =====
//       if (idx == traj_.points.size() - 1)
//       {
//         state_ = LAND;
//         RCLCPP_INFO(get_logger(), "Landing...");
//       }
//       return;
//     }

//     // ===== LAND =====
//     if (state_ == LAND)
//     {
//       land();
//       state_ = DONE;
//       return;
//     }
//   }

//   void publish_position(float x, float y, float z)
//   {
//     px4_msgs::msg::TrajectorySetpoint sp{};
//     sp.timestamp = now().nanoseconds() / 1000;
//     sp.position = { x, y, z };
//     sp.yaw = M_PI_2;
//     sp_pub_->publish(sp);
//   }

//   void publish_offboard()
//   {
//     px4_msgs::msg::OffboardControlMode msg{};
//     msg.timestamp = now().nanoseconds() / 1000;
//     msg.position = true;
//     msg.velocity = true;
//     offboard_pub_->publish(msg);
//   }
// };

// int main(int argc, char* argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<TrajectoryController>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

class TrajectoryController : public rclcpp::Node
{
public:
    TrajectoryController() : Node("trajectory_controller")
    {
        traj_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "/reference_trajectory", 10,
            std::bind(&TrajectoryController::traj_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&TrajectoryController::odom_callback, this, std::placeholders::_1));

        sp_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TrajectoryController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Trajectory Controller Started");
    }

private:
    enum State {IDLE, TAKEOFF, TRACK, LAND, DONE};
    State state_ = IDLE;

    trajectory_msgs::msg::MultiDOFJointTrajectory traj_;
    rclcpp::Time traj_start_;
    bool traj_received_ = false;
float vx_ = 0.0f;
float vy_ = 0.0f;
    float current_z_ = 0.0f;

    rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ================= CALLBACKS =================
    void traj_callback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
    {
        traj_ = *msg;
        traj_received_ = true;
        RCLCPP_INFO(get_logger(), "Trajectory received");
    }

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_z_ = msg->position[2]; // NED
        vx_ = msg->velocity[0];  // North
        vy_ = msg->velocity[1];  // East
    }

    // ================= COMMAND =================
    void send_command(uint16_t cmd, float p1=0, float p2=0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now().nanoseconds() / 1000;
        cmd_pub_->publish(msg);
    }

    void arm() { send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }

    void set_offboard() { send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); }

    void land() { send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

    float normalize_yaw(float yaw)
    {
        while (yaw > M_PI) yaw -= 2*M_PI;
        while (yaw < -M_PI) yaw += 2*M_PI;
        return yaw;
    }

    // ================= CONTROL LOOP =================
    void control_loop()
    {
        publish_offboard();

        static int counter = 0;

        // ===== IDLE → ARM =====
        if (state_ == IDLE)
        {
            if (counter++ > 20)
            {
                set_offboard();
                arm();
                state_ = TAKEOFF;
                RCLCPP_INFO(get_logger(), "TAKEOFF");
            }
            return;
        }

        // ===== TAKEOFF =====
        if (state_ == TAKEOFF)
        {
            publish_position(0, 0, -3);

            if (current_z_ < -2.8)
            {
                traj_start_ = now();
                state_ = TRACK;
                RCLCPP_INFO(get_logger(), "TRACKING");
            }
            return;
        }

        // ===== TRACK =====
        if (state_ == TRACK)
        {
            if (!traj_received_ || traj_.points.empty()) return;

            double t = (now() - traj_start_).seconds();

            size_t idx = traj_.points.size() - 1;
            for (size_t i = 0; i < traj_.points.size(); i++)
            {
                double t_pt = traj_.points[i].time_from_start.sec +
                              traj_.points[i].time_from_start.nanosec * 1e-9;

                if (t_pt >= t)
                {
                    idx = i;
                    break;
                }
            }

            auto &pt = traj_.points[idx];
            auto tr = pt.transforms[0];

            size_t next_idx = std::min(idx + 1, traj_.points.size() - 1);
            auto tr_next = traj_.points[next_idx].transforms[0];

            float dx = tr_next.translation.x - tr.translation.x;
            float dy = tr_next.translation.y - tr.translation.y;

// float yaw = 0.0f;

// if (fabs(dx) > 1e-4 || fabs(dy) > 1e-4)
// {
//     float yaw_enu = atan2f(dy, dx);

//     // ENU → NED
//     yaw = yaw_enu - M_PI_2;

//     // 🔥 CRITICAL FIX: flip forward direction
//     yaw += M_PI;

//     yaw = normalize_yaw(yaw);
// }
float yaw = 0.0f;

if (fabs(vx_) > 1e-3 || fabs(vy_) > 1e-3)
{
    yaw = atan2f(vy_, vx_);
    yaw = normalize_yaw(yaw);
}
            px4_msgs::msg::TrajectorySetpoint sp{};
            sp.timestamp = now().nanoseconds() / 1000;

            // ENU → NED
            sp.position = {
                (float)tr.translation.y,
                (float)tr.translation.x,
                (float)(-tr.translation.z)
            };

            sp.yaw = yaw;

            sp_pub_->publish(sp);

            // finish → land
            if (idx == traj_.points.size() - 1)
            {
                state_ = LAND;
                RCLCPP_INFO(get_logger(), "LAND");
            }
            return;
        }

        // ===== LAND =====
        if (state_ == LAND)
        {
            land();
            state_ = DONE;
            return;
        }
    }

    void publish_position(float x, float y, float z)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = now().nanoseconds() / 1000;
        sp.position = {x, y, z};
        sp.yaw = M_PI_2;
        sp_pub_->publish(sp);
    }

    void publish_offboard()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = now().nanoseconds() / 1000;
        msg.position = true;
        offboard_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryController>());
    rclcpp::shutdown();
    return 0;
}