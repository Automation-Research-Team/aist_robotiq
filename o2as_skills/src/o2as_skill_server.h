#ifndef O2AS_SKILL_SERVER_H
#define O2AS_SKILL_SERVER_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_listener.h>    // Includes the TF conversions
#include "o2as_helper_functions.h"

#include <chrono>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// Services
#include "o2as_skills/goToNamedPose.h"

// Actions
#include <actionlib/server/simple_action_server.h>
#include "o2as_skills/alignAction.h"
#include "o2as_skills/pickAction.h"
#include "o2as_skills/placeAction.h"
#include "o2as_skills/insertAction.h"
#include "o2as_skills/screwAction.h"

#include <actionlib/client/simple_action_client.h>
#include <robotiq_msgs/CModelCommandAction.h>

class SkillServer
{
public:
  //Constructor
  SkillServer();

  //Helpers (convenience functions)
  bool moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait = true);
  bool moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait = true);
  bool goToNamedPose(std::string pose_name, std::string robot_name);
  bool stop();                  // Stops the robot at the current position
  moveit::planning_interface::MoveGroupInterface* robotNameToMoveGroup(std::string robot_name);
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

  bool openGripper(std::string robot_name);
  bool closeGripper(std::string robot_name);
  bool sendGripperCommand(std::string robot_name, double opening_width);

  // Callback declarations
  bool goToNamedPoseCallback(o2as_skills::goToNamedPose::Request &req,
                        o2as_skills::goToNamedPose::Response &res);

  // Actions
  void executeAlign(const o2as_skills::alignGoalConstPtr& goal);
  void executePick(const o2as_skills::pickGoalConstPtr& goal);
  void executePlace(const o2as_skills::placeGoalConstPtr& goal);
  void executeInsert(const o2as_skills::insertGoalConstPtr& goal);
  void executeScrew(const o2as_skills::screwGoalConstPtr& goal);
  

private:
  ros::NodeHandle n_;

  // Service declarations
  ros::ServiceServer goToNamedPoseService_;
  
  // Action declarations
  actionlib::SimpleActionServer<o2as_skills::alignAction> alignActionServer_;
  actionlib::SimpleActionServer<o2as_skills::pickAction> pickActionServer_;
  actionlib::SimpleActionServer<o2as_skills::placeAction> placeActionServer_;
  actionlib::SimpleActionServer<o2as_skills::insertAction> insertActionServer_;
  actionlib::SimpleActionServer<o2as_skills::screwAction> screwActionServer_;  

  // Action clients
  // actionlib::SimpleActionClient<control_msgs::GripperCommandAction> a_bot_gripper_client_;
  actionlib::SimpleActionClient<robotiq_msgs::CModelCommandAction> b_bot_gripper_client_, c_bot_gripper_client_;
  

  // Status variables
  tf::TransformListener tflistener_;
  bool holding_object_ = false;
  std::string held_object_id_ = "";
  std::string held_screw_tool_ = "";    // "m3", "m4", "m5", "nut"...

  // MoveGroup connections
  moveit_msgs::PlanningScene planning_scene_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient get_planning_scene_client;
  moveit::planning_interface::MoveGroupInterface a_bot_group_, b_bot_group_, c_bot_group_, front_bots_group_, all_bots_group_;
  

  moveit_msgs::CollisionObject screw_tool_m5, screw_tool_m4, screw_tool_m3;

};//End of class SkillServer

#endif //O2AS_SKILL_SERVER_H
