#include <rclcpp/rclcpp.hpp>
#include<moveit/move_group_interface/move_group_interface.h>
#include <map>

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    auto node= std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner= std::thread([&executor]() {executor.spin();});

    auto arm= moveit::planning_interface::MoveGroupInterface(node,"arm");
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setMaxAccelerationScalingFactor(0.2);

    auto gripper= moveit::planning_interface::MoveGroupInterface(node,"gripper");
    
    //named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1)==moveit::core::MoveItErrorCode::SUCCESS);

    // if(success1)
    // {
    //     arm.execute(plan1);
    // }

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2)==moveit::core::MoveItErrorCode::SUCCESS);

    // if(success2)
    // {
    //     arm.execute(plan2);
    // }
    //---------------------------------------------------------------------------------------------

    //joint goal
    
    // std::map<std::string, double> joints = {
    //     {"base_joint", 0.5},
    //     {"shoulder_joint", 0.2},
    //     {"elbow_joint", 0.5}
    // };

    // arm.setStartStateToCurrentState();
    // const bool target_ok = arm.setJointValueTarget(joints);
    // if (!target_ok)
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to set joint target for group 'arm'.");
    //     rclcpp::shutdown();
    //     spinner.join();
    //     return 1;
    // }

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1)==moveit::core::MoveItErrorCode::SUCCESS);

    // if(success1)
    // {
    //     arm.execute(plan1);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed for arm joint goal.");
    // }
    // -------------------------------------------------------------------------
    // Joint Value Goal — bypasses IK entirely, always works for valid joints
    // Using pose_1 values from servox.srdf
    // -------------------------------------------------------------------------
    std::map<std::string, double> joint_target = {
        {"base_joint",     -0.8245},
        {"shoulder_joint",  0.3428},
        {"elbow_joint",     0.6128}
    };

    RCLCPP_INFO(node->get_logger(),
        "Joint target -> base:%.3f  shoulder:%.3f  elbow:%.3f",
        joint_target["base_joint"],
        joint_target["shoulder_joint"],
        joint_target["elbow_joint"]);

    arm.setStartStateToCurrentState();
    bool target_ok = arm.setJointValueTarget(joint_target);
    if (!target_ok)
    {
        RCLCPP_ERROR(node->get_logger(), "Invalid joint target!");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(5);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1)
    {
        RCLCPP_INFO(node->get_logger(), "Joint goal plan succeeded — executing...");
        arm.execute(plan1);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed for joint goal.");
    }

    // --- Return to home ---
    RCLCPP_INFO(node->get_logger(), "Returning to home...");
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan_home;
    bool success_home = (arm.plan(plan_home) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success_home)
    {
        arm.execute(plan_home);
        RCLCPP_INFO(node->get_logger(), "Returned to home.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed for home return.");
    }

    
    rclcpp::shutdown();
    spinner.join();
    return 0;
}