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
  
  std::string movegroup_name, ee_link1, ee_link2,ee_link3;
  geometry_msgs::PoseStamped command_cartesian_position;
  
  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "all_bots");
  nh.param<std::string>("ee_link1", ee_link1, "a_bot_robotiq_85_tip_link");
  nh.param<std::string>("ee_link2", ee_link2, "b_bot_robotiq_85_tip_link");
  nh.param<std::string>("ee_link3", ee_link3, "c_bot_robotiq_85_tip_link");
  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
    
  int bin = 1;
  std::string bin_header1, bin_header2;

  ros::Duration(1.0).sleep(); // 1 second
  
  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
      
  // Configure planner 
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link1);
  moveit::planning_interface::MoveItErrorCode success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
  motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  ros::Duration(2.0).sleep(); // 2 seconds

  geometry_msgs::PoseStamped home1, home2,home3;
  home1.pose.orientation.w = 1.0;
  home2.pose.orientation.w = 1.0;
  home3.pose.orientation.w = 1.0;
  home1.header.frame_id="a_bot_robotiq_85_tip_link";
  home2.header.frame_id="b_bot_robotiq_85_tip_link";
  home3.header.frame_id="c_bot_robotiq_85_tip_link";
  home1 = transform_pose_now(home1, "/world", tflistener);
  home2 = transform_pose_now(home2, "/world", tflistener);
  home3 = transform_pose_now(home3, "/world", tflistener);
  
  while (ros::ok()) {
    // if (my_iiwa.getRobotIsConnected()) {
    if (true) {
      bin_header1 = "/set2_bin2_1"; 
      //if (bin == 1) {bin_header1 = "/set2_bin2_1"; bin_header2 = "/set2_bin2_2"; }
      //if (bin == 2) {bin_header1 = "/set2_bin2_2"; bin_header2 = "/set2_bin2_3"; }

      geometry_msgs::PoseStamped ps1;
      ps1.pose = makePose(0, 0, .03);
      // The lines below rotate the pose so the robot looks downwards.
      rotatePoseByRPY(0.0, 0.0, M_PI/2.0, ps1.pose);
      rotatePoseByRPY(0.0, M_PI/2.0, 0.0, ps1.pose);

      //ps2.pose = ps1.pose;
      ps1.header.frame_id = bin_header1;
      //ps2.header.frame_id = bin_header2;

      // ROS_INFO_STREAM("Moving to pose: " << ps.pose.position.x << ", " << ps.pose.position.y << ", " << ps.pose.position.z
      //                 << "; " << ps.pose.orientation.x << ", " << ps.pose.orientation.y << ", " << ps.pose.orientation.z << ", " << ps.pose.orientation.w);

      group.setStartStateToCurrentState();
      if(bin==1){group.setPoseTarget(ps1, ee_link1);    group.setPoseTarget(home2, ee_link2); group.setPoseTarget(home3, ee_link3);}
      if(bin==2){group.setPoseTarget(home1, ee_link1);  group.setPoseTarget(home2, ee_link2);   group.setPoseTarget(home3, ee_link3);}
      if(bin==3){group.setPoseTarget(home1, ee_link1);  group.setPoseTarget(ps1, ee_link2); group.setPoseTarget(home3, ee_link3);}
      // if(bin==2){group.setPoseTarget(home1, ee_link1);  group.setPoseTarget(ps1, ee_link2);   group.setPoseTarget(home3, ee_link3);}
      // if(bin==3){group.setPoseTarget(home1, ee_link1);  group.setPoseTarget(home2, ee_link2); group.setPoseTarget(ps1, ee_link3);}
      //group.setPoseTarget(ps2, ee_link2);
      
      success_plan = group.plan(myplan);
      if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        motion_done = group.execute(myplan);
      }
      if (motion_done) {
        loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
      }
      bin++;
      if (bin > 3)
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
