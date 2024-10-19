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
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  move_group_interface.setMaxVelocityScalingFactor(0.15);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  const moveit::core::JointModelGroup* jmg = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // /* Move to target position */
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 0;
  // target_pose1.position.x = 0.3;
  // target_pose1.position.y = 0.4;
  // target_pose1.position.z = 0.4;
  // move_group_interface.setPoseTarget(target_pose1);

  // Ask for user 
  float x, y, z, orientation_w;
  std::cout << "Enter target position (x y z): ";
  std::cin >> x >> y >> z;

  std::cout << "Enter orientation w: ";
  std::cin >> orientation_w;

  double gripper_goal_angle, max_effort;
  std::cout << "Enter gripper parameters : goal angle [-1.57, 0], max_effort [3, 20] N*m: ";
  std::cin >> gripper_goal_angle >> max_effort;

  // Move to target position
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = orientation_w;
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  move_group_interface.setPoseTarget(target_pose1);


  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success = (move_group_interface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    move_group_interface.execute(plan1);
  }

  // Cartesinan Paths
  // ^^^^^^^^^^^^^^^^
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);
  
  geometry_msgs::Pose target_pose2 = target_pose1;

  target_pose2.position.x += 0.1;
  waypoints.push_back(target_pose2); // forward

  target_pose2.position.z -= 0.1;
  waypoints.push_back(target_pose2); // up

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threhold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threhold, trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan plan2;

  plan2.trajectory_ = trajectory;
  move_group_interface.execute(plan2);


  /* Create Gripper Action client */
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> z1_gripper_client("z1_gripper", true);

  ROS_INFO("Waiting for action server to start...");
  z1_gripper_client.waitForServer();

  /* Move Gripper */
  ROS_INFO("Move Gripper.");
  
  // "gripper_goal_angle: [-1.57, 0] rad, negative direction indicates open.\n"
  // "max_effort : [3, 20] N·m, default: 10 N·m"
  // float gripper_goal_angle, max_effort;
  // std::cout << "Enter gripper parameters : goal angle [-1.57, 0], max_effort [3, 20] N*m: ";
  // std::cin >> gripper_goal_angle >> max_effort;

  control_msgs::GripperCommandGoal goal_open;
  goal_open.command.position = gripper_goal_angle; 
  goal_open.command.max_effort = max_effort;

  z1_gripper_client.sendGoal(goal_open);

  z1_gripper_client.waitForResult();
  if(z1_gripper_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Gripper move successfully.\n\
              Current state. Angle: %f, Effort: %f",
              z1_gripper_client.getResult()->position, z1_gripper_client.getResult()->effort);
  }else{
    ROS_INFO("Gripper move successfully.\n\
              Current state. Angle: %f, Effort: %f",
              z1_gripper_client.getResult()->position, z1_gripper_client.getResult()->effort);
  }


  /* Move to saved position */
  move_group_interface.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  success = (move_group_interface.plan(plan_home) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    move_group_interface.execute(plan_home);
  }

  goal_open.command.position = 0; 
  goal_open.command.max_effort = 20;
  z1_gripper_client.sendGoal(goal_open);
  z1_gripper_client.waitForResult();

  ros::shutdown();


  return 0;
}
