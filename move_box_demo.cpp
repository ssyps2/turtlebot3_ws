#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_box_demo");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_box_handle(PLANNING_GROUP);
  move_box_handle.setMaxVelocityScalingFactor(0.15);
  move_box_handle.setMaxAccelerationScalingFactor(0.5);

  const moveit::core::JointModelGroup* jmg = move_box_handle.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /* Create Gripper Action client */
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> z1_gripper_client("z1_gripper", true);
  
  ROS_INFO("Waiting for action server to start...");
  z1_gripper_client.waitForServer();

  control_msgs::GripperCommandGoal goal_open;
  
  double gripper_goal_angle = -1.5;         // [-1.57, 0] rad, negative direction indicates open
  double max_effort = 10.0;                 // [3, 20] N·m, default: 10 N·m"

  /* Move to box handle */
  move_box_handle.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  bool success = (move_box_handle.plan(plan_home) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    move_box_handle.execute(plan_home);
  }
  ROS_INFO("Initialization End");

  /* Open gripper */
  goal_open.command.position = gripper_goal_angle; 
  goal_open.command.max_effort = max_effort;
  z1_gripper_client.sendGoal(goal_open);
  z1_gripper_client.waitForResult();
  ROS_INFO("Gripper opened");
  
  /* Move to box handle */
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.45;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.36;
  move_box_handle.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  success = (move_box_handle.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO("%d", success);

  if(success) {
    move_box_handle.execute(plan1);
  }
  ROS_INFO("Moved to box handle");

  /* Make gripper hold handle */
  gripper_goal_angle= -30/180*3.14;
  goal_open.command.position = gripper_goal_angle;
  goal_open.command.max_effort = max_effort;
  z1_gripper_client.sendGoal(goal_open);
  z1_gripper_client.waitForResult();
  ROS_INFO("Gripper hold handle");

  /* Lifting the box */
  geometry_msgs::Pose target_pose2 = target_pose1;

  target_pose2.position.z += 0.2;
  move_box_handle.setPoseTarget(target_pose2);

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  success = (move_box_handle.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO("%d", success);

  if(success){
    move_box_handle.execute(plan2);
  }
  ROS_INFO("Box lifted");


  ros::shutdown();

  return 0;
}
