#include "ros/ros.h"
#include "fvd_helper_functions.h"
#include <tf/transform_listener.h>    // Includes the TF conversions
#include <moveit/move_group_interface/move_group_interface.h>

// This example moves to different positions defined by frames in the scene, 
// and shows how to transform between frames using a simple helper function.

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");
  
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // iiwa_ros::iiwaRos my_iiwa;
  // my_iiwa.init();

  tf::TransformListener tflistener;
  
  std::string movegroup_name, ee_link;
  geometry_msgs::PoseStamped command_cartesian_position;
  
  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "a_bot");
  nh.param<std::string>("ee_link", ee_link, "a_bot_ee_link");
  
  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
    
  int bin = 1;
  std::string bin_header;

  ros::Duration(1.0).sleep(); // 1 second
  
  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
      
  // Configure planner 
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link);
  moveit::planning_interface::MoveItErrorCode success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
  motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  ros::Duration(2.0).sleep(); // 2 seconds
  while (ros::ok()) {
    // if (my_iiwa.getRobotIsConnected()) {
    if (true) {
      
      if (bin == 1) {bin_header = "/set2_bin1";}
      if (bin == 2) {bin_header = "/set2_bin2";}
      if (bin == 3) {bin_header = "/set2_bin3";}
      if (bin == 4) {bin_header = "/set2_bin4";}

      geometry_msgs::PoseStamped ps;
      ps.pose = makePose(0, 0, .1);
      ps.header.frame_id = bin_header;
      transform_pose_now(ps, "/a_bot_base_link", tflistener);
      // The above line is an example of an easy way to transform between frames, 
      // but I am not sure it is necessary here, since MoveIt already accepts stamped poses.

      // The lines below rotate the pose so the robot looks downwards.
      rotatePoseByRPY(0.0, 0.0, M_PI/2.0, ps.pose);
      rotatePoseByRPY(0.0, M_PI/2.0, 0.0, ps.pose);

      ROS_INFO_STREAM("Moving to pose: " << ps.pose.position.x << ", " << ps.pose.position.y << ", " << ps.pose.position.z
                      << "; " << ps.pose.orientation.x << ", " << ps.pose.orientation.y << ", " << ps.pose.orientation.z << ", " << ps.pose.orientation.w);

      group.setStartStateToCurrentState();
      group.setPoseTarget(ps, ee_link);
      success_plan = group.plan(myplan);
      if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        motion_done = group.execute(myplan);
      }
      if (motion_done) {
        loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
      }
      bin++;
      if (bin > 4)
      {
        bin = 1;
      }
    }
    else {
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
    }
  }  
}; 