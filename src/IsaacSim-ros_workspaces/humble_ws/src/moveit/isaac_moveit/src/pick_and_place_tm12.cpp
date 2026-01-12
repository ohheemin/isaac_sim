#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/float64.hpp>

#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

enum class J1Direction
{
    ANY,
    INCREASE,
    DECREASE
};

class MoveItToJointCommand : public rclcpp::Node
{
public:
    MoveItToJointCommand() : Node("moveit_to_jointcommand")
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);

        pickplace_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pickandplace", 10);

        placepick_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/placeandpick", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/isaac_joint_states",
            10,
            std::bind(&MoveItToJointCommand::jointStateCallback, this, std::placeholders::_1));

        pickplace_timer_ = this->create_wall_timer(
            0.0001ms, std::bind(&MoveItToJointCommand::publishJoint1, this));

        RCLCPP_INFO(this->get_logger(),
            "MoveIt Bridge Node Started. Subscribing to /isaac_joint_states");
    }

    void initialize()
    {
        move_group_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "tmr_arm");

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        waitForFirstJointState();

        tf2::Quaternion q;
        q.setRPY(M_PI, 0, M_PI);

        geometry_msgs::msg::Pose pick_pose1;
        pick_pose1.position.x = -0.235;
        pick_pose1.position.y = 0.5;
        pick_pose1.position.z = 0.275;
        pick_pose1.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose pick_approach1 = pick_pose1;
        pick_approach1.position.z += 0.2;

        geometry_msgs::msg::Pose place_pose1;
        place_pose1.position.x = -0.235;
        place_pose1.position.y = -0.7;
        place_pose1.position.z = 0.43;
        place_pose1.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose place_approach1 = place_pose1;
        place_approach1.position.z += 0.2;

        RCLCPP_WARN(this->get_logger(), "Pick Place Sequence Started");

        run_sequence(pick_approach1, J1Direction::INCREASE);
        std::this_thread::sleep_for(7s);

        cartesian_move(pick_pose1);
        std::this_thread::sleep_for(7s);

        cartesian_move(pick_approach1);
        std::this_thread::sleep_for(7s);

        run_sequence(place_approach1, J1Direction::INCREASE);
        std::this_thread::sleep_for(7s);

        cartesian_move(place_pose1);
        std::this_thread::sleep_for(7s);

        cartesian_move(place_approach1);
        std::this_thread::sleep_for(7s);

        RCLCPP_WARN(this->get_logger(), "Pick & Place Completed");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pickplace_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr placepick_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr pickplace_timer_;

    std::mutex joint_mutex_;
    std::vector<double> current_joints_;
    bool have_joint_state_ = false;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (msg->position.size() >= 6)
        {
            current_joints_ = msg->position;
            have_joint_state_ = true;
        }
    }

    void publishJoint1()
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (!have_joint_state_) return;

        double j1 = current_joints_[0];
        std_msgs::msg::Float64 msg;
        msg.data = j1;

        if (j1 > 2.29 && j1 < 4.6)
        {
            pickplace_pub_->publish(msg);
        }
        else if (j1 >= 4.6)
        {
            placepick_pub_->publish(msg);
        }
    }

    void waitForFirstJointState()
    {
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && !have_joint_state_)
            rate.sleep();
    }

    bool cartesian_move(const geometry_msgs::msg::Pose& target)
    {
        move_group_->setStartStateToCurrentState();

        std::vector<geometry_msgs::msg::Pose> waypoints{target};
        moveit_msgs::msg::RobotTrajectory traj;

        double fraction = move_group_->computeCartesianPath(
            waypoints, 0.01, 0.0, traj);

        if (fraction < 0.95)
            return false;

        playTrajectory(traj.joint_trajectory);
        return true;
    }

    std::vector<double> plan_and_publish(
        const geometry_msgs::msg::Pose& pose,
        J1Direction j1_dir)
    {
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(pose);

        double start_j1 = current_joints_[0];

        for (int attempt = 0; attempt < 10; attempt++)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_->plan(plan) !=
                moveit::core::MoveItErrorCode::SUCCESS)
                continue;

            bool valid = true;
            for (auto& p : plan.trajectory_.joint_trajectory.points)
            {
                double j1 = p.positions[0];
                double j3 = p.positions[2];
                double j4 = p.positions[3];

                if (j3 < 0.0 || j4 > 0.0)
                    valid = false;

                if (j1_dir == J1Direction::INCREASE && j1 < start_j1)
                    valid = false;

                if (j1_dir == J1Direction::DECREASE && j1 > start_j1)
                    valid = false;
            }

            if (!valid) continue;

            playTrajectory(plan.trajectory_.joint_trajectory);
            return plan.trajectory_.joint_trajectory.points.back().positions;
        }

        RCLCPP_FATAL(this->get_logger(), "FAILED to find valid trajectory");
        return {};
    }

    void run_sequence(const geometry_msgs::msg::Pose& pose, J1Direction dir)
    {
        plan_and_publish(pose, dir);
    }

    void playTrajectory(const trajectory_msgs::msg::JointTrajectory& traj)
    {
        for (auto& p : traj.points)
        {
            publish_joint_command(p.positions);
            std::this_thread::sleep_for(10ms);
        }
    }

    void publish_joint_command(const std::vector<double>& joints)
    {
        sensor_msgs::msg::JointState msg;
        msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
        msg.position = joints;
        msg.header.stamp = now();
        joint_pub_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItToJointCommand>();

    std::thread([node]() {
        std::this_thread::sleep_for(3s);
        node->initialize();
    }).detach();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
