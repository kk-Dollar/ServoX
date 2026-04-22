#include <rclcpp/rclcpp.hpp>
#include<moveit/move_group_interface/move_group_interface.h>
#include <algorithm>
#include <example_interfaces/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using BoolMsg = example_interfaces::msg::Bool;
using StringMsg = std_msgs::msg::String;
using JointStateMsg = sensor_msgs::msg::JointState;
using PoseMsg = geometry_msgs::msg::Pose;

class Commander
{
public:
  Commander(std::shared_ptr<rclcpp::Node> node)
  {
    node_=node;
    arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    arm_->setMaxAccelerationScalingFactor(1.0);
    arm_->setMaxVelocityScalingFactor(1.0);
    gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

    open_gripper_sub_ = node_->create_subscription<BoolMsg>(
      "open_gripper", 10, std::bind(&Commander::openGripperCallback, this, std::placeholders::_1));

    joint_target_sub_ = node_->create_subscription<JointStateMsg>(
      "joint_target", 10, std::bind(&Commander::jointTargetCallback, this, std::placeholders::_1));

    named_pose_sub_ = node_->create_subscription<StringMsg>(
      "named_pose", 10, std::bind(&Commander::namedPoseCallback, this, std::placeholders::_1));

    pose_target_sub_ = node_->create_subscription<PoseMsg>(
      "pose_target", 10, std::bind(&Commander::poseTargetCallback, this, std::placeholders::_1));
  }
  void goToNamedTarget(const std::string &name)
  {
    arm_->setStartStateToCurrentState();
    arm_->setNamedTarget(name);
    planAndExecute(arm_);
  }
  void goToJointTarget(const std::map<std::string, double> &joints)
  {
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(joints);
    planAndExecute(arm_);
  }
  void openGripper()
  {
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget("gripper_open");
    planAndExecute(gripper_);
  }
  void closeGripper()
  {
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget("gripper_closed");
    planAndExecute(gripper_);
  }
  void goToPoseTarget(const PoseMsg &pose)
  {
    arm_->setStartStateToCurrentState();

    // 3-DOF arm: command Cartesian position only; orientation is underconstrained.
    arm_->setPositionTarget(pose.position.x, pose.position.y, pose.position.z);
    planAndExecute(arm_);
  }

private:
  
  void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
  {
    MoveGroupInterface::Plan plan;
    bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success)
    {
      interface->execute(plan);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed for group '%s'.", interface->getName().c_str());
    }
  }

  void openGripperCallback(const BoolMsg &msg)
  {
    if(msg.data)
    {
      openGripper();
    }
    else
    {
      closeGripper();
    }
  }

  void jointTargetCallback(const JointStateMsg &msg)
  {
    const auto arm_joint_names = arm_->getJointNames();

    if(msg.name.size() != msg.position.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint target: name/position size mismatch.");
      return;
    }
    if(msg.name.size() != arm_joint_names.size())
    {
      RCLCPP_ERROR(node_->get_logger(),
        "Joint target: expected %zu joints (arm group), got %zu.",
        arm_joint_names.size(), msg.name.size());
      return;
    }
    std::map<std::string, double> joint_map;
    for(size_t i = 0; i < msg.name.size(); ++i)
    {
      if(std::find(arm_joint_names.begin(), arm_joint_names.end(), msg.name[i]) == arm_joint_names.end())
      {
        RCLCPP_ERROR(node_->get_logger(),
          "Joint '%s' is not part of the arm group. Arm joints: base_joint, shoulder_joint, elbow_joint.",
          msg.name[i].c_str());
        return;
      }
      joint_map[msg.name[i]] = msg.position[i];
    }
    RCLCPP_INFO(node_->get_logger(), "Received joint target with %zu joints.", joint_map.size());
    goToJointTarget(joint_map);
  }

  void namedPoseCallback(const StringMsg &msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Received named pose: '%s'", msg.data.c_str());
    goToNamedTarget(msg.data);
  }

  void poseTargetCallback(const PoseMsg &msg)
  {
    RCLCPP_INFO(node_->get_logger(),
      "Received pose target position: [x=%.3f, y=%.3f, z=%.3f]",
      msg.position.x, msg.position.y, msg.position.z);
    goToPoseTarget(msg);
  }
  
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;
  std::shared_ptr<MoveGroupInterface> gripper_;

  rclcpp::Subscription<BoolMsg>::SharedPtr open_gripper_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_target_sub_;
  rclcpp::Subscription<StringMsg>::SharedPtr named_pose_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_target_sub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("servox_commander");
    auto commander= Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}