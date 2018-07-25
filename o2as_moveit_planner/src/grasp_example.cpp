#include "ros/ros.h"
#include "fvd_helper_functions.h"
#include <tf/transform_listener.h>    // Includes the TF conversions
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/Grasp.h>

// This example spawns an object in the scene and tries to pick and place it. Based on:
// http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
// http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html

// ################################################################

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add the knuckle joint of the robotiq gripper. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "a_bot_robotiq_85_left_knuckle_joint";

  /* Set it to some open position, whatever this unit is supposed to be. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add the knuckle joint of the robotiq gripper. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "a_bot_robotiq_85_left_knuckle_joint";

  /* Set to closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.5;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // ROS_INFO("Moving to a pre-set pose to check validity.");
  // // Start by moving the robot to a nearby pose to check if the pose works at all.
  // geometry_msgs::PoseStamped ps;
  // ps.header.frame_id = "set3_tray_2";
  // ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/12, M_PI);
  // ps.pose.position.x = 0;
  // ps.pose.position.y = 0;
  // ps.pose.position.z = 0.15;

  // moveit::planning_interface::MoveGroupInterface::Plan myplan;
  // move_group.setStartStateToCurrentState();
  // move_group.setPoseTarget(ps, "a_bot_robotiq_85_tip_link");
  // moveit::planning_interface::MoveItErrorCode success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;
  // success_plan = move_group.plan(myplan);
  // if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
  //   motion_done = move_group.execute(myplan);
  // }
  // else{ROS_WARN("Did not manage to plan to the pose.");}


  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(20);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of the parent link of the robot's end effector.
  // The orientation is not tested yet, but hopefully looks down.
  grasps[0].grasp_pose.header.frame_id = "set3_tray_2";
  grasps[0].grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/12, M_PI);
  grasps[0].grasp_pose.pose.position.x = 0.0;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.15;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "set3_tray_2";
  /* Direction is set as negative x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "set3_tray_2";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Fill the Grasp msg with the eef posture before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);

  // Fill the Grasp msg with the eef posture during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);

  for (int i=1;i<20;i++)
  {
    grasps[i] = grasps[0];
    // grasps[i].grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/8 + rand()*M_PI/12, M_PI);
    // grasps[i].grasp_pose.pose.position.x = grasps[i].grasp_pose.pose.position.x + rand()*.02;
    // grasps[i].grasp_pose.pose.position.y = grasps[i].grasp_pose.pose.position.y;
    // grasps[i].grasp_pose.pose.position.z = grasps[i].grasp_pose.pose.position.z + rand()*.02;
  }

  // Set support surface as table1.
  move_group.setSupportSurfaceName("set3_tray_2");  // TODO: This might have to be an object in the planning scene, not the robot definition
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(20);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "set3_tray_2";
  place_location[0].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, 0.0, -M_PI/2);
  rotatePoseByRPY(0.0, 0.0, M_PI/2.0, place_location[0].place_pose.pose);

  /* This pose refers to the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0;
  place_location[0].place_pose.pose.position.z = 0.22;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "set3_tray_2";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "set3_tray_2";
  /* Direction is set as z axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  for (int i=1;i<20;i++)
  {
    place_location[i] = place_location[0];
  }

  // Set support surface as table2.
  group.setSupportSurfaceName("set3_tray_2"); // TODO: This might have to be an object in the planning scene, not the robot definition
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, bool already_published_object)
{
  if (!already_published_object)
  {
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold a collision object.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Define the object that we will be manipulating
    collision_objects[0].header.frame_id = "set3_tray_2";
    collision_objects[0].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.02;
    collision_objects[0].primitives[0].dimensions[1] = 0.02;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.21;

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
  }
}

// ################################################################

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");
  
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  tf::TransformListener tflistener;
  
  std::string movegroup_name, ee_link;
  bool already_published_object;
  nh.param<std::string>("move_group", movegroup_name, "a_bot");
  nh.param<std::string>("ee_link", ee_link, "a_bot_robotiq_85_tip_link");
  nh.param<bool>("already_published_object", already_published_object, false);
  
  // Dynamic parameter to choose the rate at which this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
    
  std::string bin_header;

  ros::Duration(1.0).sleep(); // 1 second
  
  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group_a(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  
  // Configure planner 
  // group_a.setPlanningTime(0.5);
  group_a.setPlannerId("RRTConnectkConfigDefault");
  group_a.setEndEffectorLink(ee_link);
  moveit::planning_interface::MoveItErrorCode success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
  motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  ros::Duration(2.0).sleep(); // 2 seconds

  ///////
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  group_a.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface, already_published_object);

  ROS_INFO("Attempting to pick the object.");
  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group_a);

  ROS_INFO("Attempting to place the object.");
  ros::WallDuration(1.0).sleep();

  place(group_a);
    
  ros::waitForShutdown();
  return 0;   
}; 
