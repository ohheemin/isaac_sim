#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class MoveItToJointCommand : public rclcpp::Node
{
public:
    MoveItToJointCommand() : Node("moveit_to_jointcommand")
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);

        RCLCPP_INFO(this->get_logger(), "/joint_command bridge started.");
    }

    void initialize()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt...");

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "panda_arm");

        move_group_->setPlanningTime(8.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        tf2::Quaternion q;
        q.setRPY(M_PI, 0, 0.785398); // yaw 45deg

        geometry_msgs::msg::Pose pick_pose1;
        pick_pose1.position.x = -0.235;
        pick_pose1.position.y = 0.5;
        pick_pose1.position.z = 0.16;
        pick_pose1.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose pick_approach1 = pick_pose1;
        pick_approach1.position.z += 0.2;

        geometry_msgs::msg::Pose place_pose1;
        place_pose1.position.x = 0.5;
        place_pose1.position.y = -0.2;
        place_pose1.position.z = 0.16;
        place_pose1.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose place_approach1 = place_pose1;
        place_approach1.position.z += 0.2;

        run_sequence(pick_approach1);
        auto j2 = run_sequence(pick_pose1);
        close_gripper(j2);

        run_sequence(pick_approach1);
        run_sequence(place_approach1);
        auto j5 = run_sequence(place_pose1);
        open_gripper(j5);
        run_sequence(place_approach1);

        geometry_msgs::msg::Pose pick_pose2;
        pick_pose2.position.x = 0.28;
        pick_pose2.position.y = 0.5;
        pick_pose2.position.z = 0.16;
        pick_pose2.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose pick_approach2 = pick_pose2;
        pick_approach2.position.z += 0.2;

        geometry_msgs::msg::Pose place_pose2;
        place_pose2.position.x = 0.6;
        place_pose2.position.y = -0.2;
        place_pose2.position.z = 0.16;
        place_pose2.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose place_approach2 = place_pose2;
        place_approach2.position.z += 0.2;

        run_sequence(pick_approach2);
        auto j7 = run_sequence(pick_pose2);
        close_gripper(j7);

        run_sequence(pick_approach2);
        run_sequence(place_approach2);
        auto j10 = run_sequence(place_pose2);
        open_gripper(j10);
        run_sequence(place_approach2);

        RCLCPP_INFO(this->get_logger(), "Pick & Place 완료");
    }

private:
    double current_gripper_ = 0.04;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    double computeYaw(const geometry_msgs::msg::Pose& pose)
    {
        return std::atan2(pose.position.y, pose.position.x);
    }

    void playTrajectory(const trajectory_msgs::msg::JointTrajectory& traj)
    {
        for(size_t i = 0; i < traj.points.size(); i++)
        {
            auto& p = traj.points[i];

            publish_joint_command(p.positions);

            if(i + 1 < traj.points.size())
            {
                double t1 = rclcpp::Duration(p.time_from_start).seconds();
                double t2 = rclcpp::Duration(traj.points[i+1].time_from_start).seconds();
                double dt = t2 - t1;

                if(dt < 0.001)
                    dt = 0.001;

                std::this_thread::sleep_for(std::chrono::duration<double>(dt));
            }
        }
    }

    std::vector<double> rotate_base_to_yaw(double yaw_target)
    {
        auto current = move_group_->getCurrentJointValues();
        current[0] = yaw_target;

        move_group_->setStartStateToCurrentState();
        move_group_->setJointValueTarget(current);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if(move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            playTrajectory(plan.trajectory_.joint_trajectory);
            return plan.trajectory_.joint_trajectory.points.back().positions;
        }

        RCLCPP_ERROR(this->get_logger(), "Base rotation planning failed");
        return {};
    }

    void lock_base_joint(double yaw_fixed)
    {
        moveit_msgs::msg::Constraints constraints;
        moveit_msgs::msg::JointConstraint jc;

        jc.joint_name = "panda_joint1";
        jc.position = yaw_fixed;
        jc.tolerance_above = 0.005;
        jc.tolerance_below = 0.005;
        jc.weight = 1.0;

        constraints.joint_constraints.push_back(jc);
        move_group_->setPathConstraints(constraints);
        move_group_->setPlanningTime(8.0);
    }

    std::vector<double> plan_and_publish(const geometry_msgs::msg::Pose& pose)
    {
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan best_plan;
        size_t min_points = SIZE_MAX;

        const int num_attempts = 6;

        for(int i=0;i<num_attempts;i++)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto error = move_group_->plan(plan);

            if(error != moveit::core::MoveItErrorCode::SUCCESS)
                continue;

            size_t n_points = plan.trajectory_.joint_trajectory.points.size();

            RCLCPP_INFO(this->get_logger(),
                        "[PLAN %d] trajectory points = %zu", i, n_points);

            if(n_points > 25)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Trajectory points = %zu > 25, replanning...", n_points);
                continue;
            }

            if(n_points < min_points)
            {
                min_points = n_points;
                best_plan = plan;
            }
        }

        if(min_points == SIZE_MAX)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveIt planning failed.");
            return {};
        }

        auto traj = best_plan.trajectory_.joint_trajectory;

        playTrajectory(traj);

        return traj.points.back().positions;
    }

    std::vector<double> run_sequence(const geometry_msgs::msg::Pose& pose)
    {
        double yaw = computeYaw(pose);

        rotate_base_to_yaw(yaw);
        lock_base_joint(yaw);
        auto result = plan_and_publish(pose);

        move_group_->clearPathConstraints();
        return result;
    }

    void publish_joint_command(const std::vector<double>& arm_joints)
    {
        sensor_msgs::msg::JointState msg;

        msg.name = {
            "panda_joint1","panda_joint2","panda_joint3",
            "panda_joint4","panda_joint5","panda_joint6",
            "panda_joint7",
            "panda_finger_joint1","panda_finger_joint2"
        };

        msg.position = arm_joints;

        if(msg.position.size() < 9)
        {
            msg.position.push_back(current_gripper_);
            msg.position.push_back(current_gripper_);
        }

        msg.velocity.resize(9, 0.0);
        msg.effort.resize(9, 0.0);

        msg.header.stamp = now();
        joint_pub_->publish(msg);
    }

    void close_gripper(const std::vector<double>& last_joints)
    {
        current_gripper_ = 0.0;
        publish_fixed_gripper(last_joints);
        RCLCPP_INFO(this->get_logger(), "Gripper closed.");
    }

    void open_gripper(const std::vector<double>& last_joints)
    {
        current_gripper_ = 0.04;
        publish_fixed_gripper(last_joints);
        RCLCPP_INFO(this->get_logger(), "Gripper opened.");
    }

    void publish_fixed_gripper(const std::vector<double>& last_joints)
    {
        sensor_msgs::msg::JointState msg;

        msg.name = {
            "panda_joint1","panda_joint2","panda_joint3",
            "panda_joint4","panda_joint5","panda_joint6",
            "panda_joint7",
            "panda_finger_joint1","panda_finger_joint2"
        };

        msg.position = last_joints;
        msg.position.push_back(current_gripper_);
        msg.position.push_back(current_gripper_);

        msg.velocity.resize(9, 0.0);
        msg.effort.resize(9, 0.0);

        msg.header.stamp = now();
        joint_pub_->publish(msg);

        std::this_thread::sleep_for(150ms);

        msg.header.stamp = now();
        joint_pub_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveItToJointCommand>();

    std::thread([node]() {
        std::this_thread::sleep_for(2s);
        node->initialize();
    }).detach();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
