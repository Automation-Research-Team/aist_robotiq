#ifndef O2AS_SKILL_SERVER_H
#define O2AS_SKILL_SERVER_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_listener.h>    // Includes the TF conversions
#include <tf/transform_broadcaster.h>

#include "ur_modern_driver/wait_for_program.h"
#include "o2as_helper_functions.h"

#include <chrono>
#include <thread>
#include <cmath>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

#include "visualization_msgs/Marker.h"

// Services
#include "o2as_msgs/goToNamedPose.h"
#include "o2as_msgs/sendScriptToUR.h"
#include "o2as_msgs/publishMarker.h"
#include "o2as_msgs/PrecisionGripperCommand.h"

// Actions
#include <actionlib/server/simple_action_server.h>
#include "o2as_msgs/alignAction.h"
#include "o2as_msgs/pickAction.h"
#include "o2as_msgs/placeAction.h"
#include "o2as_msgs/regraspAction.h"
#include "o2as_msgs/insertAction.h"
#include "o2as_msgs/screwAction.h"

#include <actionlib/client/simple_action_client.h>
#include <robotiq_msgs/CModelCommandAction.h>

class SkillServer
{
public:
  //Constructor
  SkillServer();

  //Helpers (convenience functions)
  bool moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait = true, std::string end_effector_link = "", double velocity_scaling_factor = 0.1);
  bool moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait = true, std::string end_effector_link = "", double velocity_scaling_factor = 0.1);
  bool goToNamedPose(std::string pose_name, std::string robot_name);
  bool stop();                  // Stops the robot at the current position
  moveit::planning_interface::MoveGroupInterface* robotNameToMoveGroup(std::string robot_name);
  std::string getEELink(std::string robot_name);
  bool updatePlanningScene();

  // Internal functions
  bool equipScrewTool(std::string robot_name, std::string screw_tool_id);
  bool unequipScrewTool(std::string robot_name);
  bool equipUnequipScrewTool(std::string robot_name, std::string screw_tool_id, std::string equip_or_unequip);
  bool spawnTool(std::string screw_tool_id);
  bool despawnTool(std::string screw_tool_id);
  bool attachTool(std::string screw_tool_id, std::string link_name);
  bool detachTool(std::string screw_tool_id, std::string link_name);
  bool attachDetachTool(std::string screw_tool_id, std::string link_name, std::string attach_or_detach);
  bool goFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, double velocity_scaling_factor);
  bool placeFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name = "");
  bool pickFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name = "");
  bool pickScrew(std::string object_id, std::string screw_tool_id, std::string robot_name);
  bool publishMarker(geometry_msgs::PoseStamped marker_pose, std::string marker_type = "");
  bool publishPoseMarker(geometry_msgs::PoseStamped marker_pose);

  bool openGripper(std::string robot_name, std::string gripper_name = "");
  bool closeGripper(std::string robot_name, std::string gripper_name = "");
  bool sendGripperCommand(std::string robot_name, double opening_width, std::string gripper_name = "");

  // Callback declarations
  bool goToNamedPoseCallback(o2as_msgs::goToNamedPose::Request &req,
                        o2as_msgs::goToNamedPose::Response &res);
  bool publishMarkerCallback(o2as_msgs::publishMarker::Request &req,
                        o2as_msgs::publishMarker::Response &res);

  // Actions
  void executeAlign(const o2as_msgs::alignGoalConstPtr& goal);
  void executePick(const o2as_msgs::pickGoalConstPtr& goal);
  void executePlace(const o2as_msgs::placeGoalConstPtr& goal);
  void executeRegrasp(const o2as_msgs::regraspGoalConstPtr& goal);
  void executeInsert(const o2as_msgs::insertGoalConstPtr& goal);
  void executeScrew(const o2as_msgs::screwGoalConstPtr& goal);
  

// private:
  ros::NodeHandle n_;

  ros::Publisher pubMarker_;
  int marker_id_count = 0;

  // Service declarations
  ros::ServiceServer goToNamedPoseService_;
  ros::ServiceServer publishMarkerService_;

  // Service clients
  ros::ServiceClient sendScriptToURClient_;
  ros::ServiceClient PrecisionGripperClient_;
  
  // Action declarations
  actionlib::SimpleActionServer<o2as_msgs::alignAction> alignActionServer_;
  actionlib::SimpleActionServer<o2as_msgs::pickAction> pickActionServer_;
  actionlib::SimpleActionServer<o2as_msgs::placeAction> placeActionServer_;
  actionlib::SimpleActionServer<o2as_msgs::regraspAction> regraspActionServer_;
  actionlib::SimpleActionServer<o2as_msgs::insertAction> insertActionServer_;
  actionlib::SimpleActionServer<o2as_msgs::screwAction> screwActionServer_;  

  // Action clients
  // actionlib::SimpleActionClient<control_msgs::GripperCommandAction> a_bot_gripper_client_;
  actionlib::SimpleActionClient<robotiq_msgs::CModelCommandAction> b_bot_gripper_client_, c_bot_gripper_client_;

  double PLANNING_TIME = 5.0, LIN_PLANNING_TIME = 15.0;
  
  // Status variables
  tf::TransformListener tflistener_;
  tf::TransformBroadcaster tfbroadcaster_;
  bool holding_object_ = false;
  // A status of the robot. This should almost definitely be rosparams instead.
  // /a_bot/status/carrying_object  (bool)
  // /a_bot/status/carrying_tool    (bool)
  // /a_bot/status/held_object_id   (string)
  // /a_bot/status/held_tool_id     (string)
  // std::map<std::string robot_name, std::map<std::string attribute, std::string status>> robot_statuses;
  std::string held_object_id_ = "";
  std::string held_screw_tool_ = "";    // "m3", "m4", "m5", "nut"...

  // MoveGroup connections
  moveit_msgs::PlanningScene planning_scene_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient get_planning_scene_client;
  moveit::planning_interface::MoveGroupInterface a_bot_group_, b_bot_group_, c_bot_group_; // front_bots_group_, all_bots_group_;
  
  moveit_msgs::CollisionObject screw_tool_m5, screw_tool_m4, screw_tool_m3;

};//End of class SkillServer

#endif //O2AS_SKILL_SERVER_H
