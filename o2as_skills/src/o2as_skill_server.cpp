#include "o2as_skill_server.h"

SkillServer::SkillServer() : 
                  alignActionServer_(n_, "o2as_skills/align", boost::bind(&SkillServer::executeAlign, this, _1),false),
                  pickActionServer_(n_, "o2as_skills/pick", boost::bind(&SkillServer::executePick, this, _1),false),
                  placeActionServer_(n_, "o2as_skills/place", boost::bind(&SkillServer::executePlace, this, _1),false),
                  insertActionServer_(n_, "o2as_skills/insert", boost::bind(&SkillServer::executeInsert, this, _1),false),
                  screwActionServer_(n_, "o2as_skills/screw", boost::bind(&SkillServer::executeScrew, this, _1),false),
                  a_bot_group_("a_bot"), b_bot_group_("b_bot"), c_bot_group_("c_bot"),
                  front_bots_group_("front_bots"), all_bots_group_("all_bots"),
                  b_bot_gripper_client_("/b_bot_gripper/gripper_action_controller", true),
                  c_bot_gripper_client_("/c_bot_gripper/gripper_action_controller", true)
                  // a_bot_gripper_client_("/a_bot_gripper/gripper_action_controller", true),
{ 
  // Topics to publish

  // Services to advertise
  goToNamedPoseService_ = n_.advertiseService("o2as_skills/goToNamedPose", &SkillServer::goToNamedPoseCallback,
                                        this);

  // Actions we serve
  alignActionServer_.start();
  pickActionServer_.start();
  placeActionServer_.start();
  insertActionServer_.start();
  screwActionServer_.start();

  // Action clients
  // ROS_INFO("Waiting for action servers to start.");
  // // a_bot_gripper_client.waitForServer(); 
  // b_bot_gripper_client_.waitForServer();
  // c_bot_gripper_client_.waitForServer();
  // ROS_INFO("Action servers started.");

  // Set up MoveGroups
  a_bot_group_.setPlanningTime(1.0);
  a_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  a_bot_group_.setEndEffectorLink("a_bot_robotiq_85_tip_link");
  b_bot_group_.setPlanningTime(1.0);
  b_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  b_bot_group_.setEndEffectorLink("b_bot_robotiq_85_tip_link");
  c_bot_group_.setPlanningTime(1.0);
  c_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  c_bot_group_.setEndEffectorLink("c_bot_robotiq_85_tip_link");
  front_bots_group_.setPlanningTime(2.0);
  front_bots_group_.setPlannerId("RRTConnectkConfigDefault");
  all_bots_group_.setPlanningTime(3.0);
  all_bots_group_.setPlannerId("RRTConnectkConfigDefault");

  // --- Define the screw tools.
  
  // Define one of the tools as a collision object
  screw_tool_m5.header.frame_id = "screw_tool_m5_link";
  screw_tool_m5.id = "screw_tool_m5";

  screw_tool_m5.primitives.resize(3);
  screw_tool_m5.primitive_poses.resize(3);
  // The bit cushion and motor
  screw_tool_m5.primitives[0].type = screw_tool_m5.primitives[0].BOX;
  screw_tool_m5.primitives[0].dimensions.resize(3);
  screw_tool_m5.primitives[0].dimensions[0] = 0.026;
  screw_tool_m5.primitives[0].dimensions[1] = 0.04;
  screw_tool_m5.primitives[0].dimensions[2] = 0.055;
  screw_tool_m5.primitive_poses[0].position.x = 0;
  screw_tool_m5.primitive_poses[0].position.y = -0.009;
  screw_tool_m5.primitive_poses[0].position.z = 0.0275;

  // The "shaft" + suction attachment
  screw_tool_m5.primitives[1].type = screw_tool_m5.primitives[1].BOX;
  screw_tool_m5.primitives[1].dimensions.resize(3);
  screw_tool_m5.primitives[1].dimensions[0] = 0.02;
  screw_tool_m5.primitives[1].dimensions[1] = 0.028;
  screw_tool_m5.primitives[1].dimensions[2] = 0.08;
  screw_tool_m5.primitive_poses[1].position.x = 0;
  screw_tool_m5.primitive_poses[1].position.y = -0.0055;  // 21 mm distance from axis
  screw_tool_m5.primitive_poses[1].position.z = -0.04;

  // The cylinder representing the tip
  screw_tool_m5.primitives[2].type = screw_tool_m5.primitives[2].CYLINDER;
  screw_tool_m5.primitives[2].dimensions.resize(2);
  screw_tool_m5.primitives[2].dimensions[0] = 0.02;    // Cylinder height
  screw_tool_m5.primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
  screw_tool_m5.primitive_poses[2].position.x = 0;
  screw_tool_m5.primitive_poses[2].position.y = 0;  // 21 mm distance from axis
  screw_tool_m5.primitive_poses[2].position.z = -0.09;

  screw_tool_m5.operation = screw_tool_m5.ADD;
  
  // The other tools
  screw_tool_m4 = screw_tool_m5;
  screw_tool_m3 = screw_tool_m5;
}


// This works only for a single robot.
bool SkillServer::moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait)
{
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);

  group_pointer->setStartStateToCurrentState();
  group_pointer->setPoseTarget(pose);

  ROS_DEBUG_STREAM("Planning motion for robot " << robot_name << ".");
  success_plan = group_pointer->plan(myplan);
  if (success_plan) 
  {
    if (wait) motion_done = group_pointer->execute(myplan);
    else motion_done = group_pointer->asyncExecute(myplan);
    if (motion_done) {
      return true;
    }
  }
  ROS_WARN("Failed to perform motion.");
  return false;
}

// This works only for a single robot.
bool SkillServer::moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait)
{
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);

  group_pointer->setStartStateToCurrentState();
  
  // Plan cartesian motion
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped start_pose;
  start_pose.header.frame_id = robot_name + "_robotiq_85_tip_link";
  start_pose.pose = makePose();
  start_pose = transform_pose_now(start_pose, "world", tflistener_);
  pose = transform_pose_now(pose, "world", tflistener_);
  waypoints.push_back(start_pose.pose);
  waypoints.push_back(pose.pose);

  group_pointer->setMaxVelocityScalingFactor(0.1);
  b_bot_group_.setPlanningTime(5.0);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double cartesian_success = group_pointer->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("Cartesian motion plan was %.2f%% successful.", cartesian_success * 100.0);

  // //----- Visualize the plan in RViz
  // ROS_INFO("Visualizing cartesian motion into screw holder (%.2f%% achieved)", cartesian_success * 100.0);
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools(robot_name + "_base_link");
  // visual_tools.deleteAllMarkers();
  // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  // text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools.trigger();
  // //-----

  // Does the start state of the plan need to be set?
  myplan.trajectory_ = trajectory;
  // if (cartesian_success > .95) 
  if (true) 
  {
    if (wait) motion_done = group_pointer->execute(myplan);
    else motion_done = group_pointer->asyncExecute(myplan);
    if (motion_done) 
    {
      group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
      b_bot_group_.setPlanningTime(1.0);
      return true;
    }
  }
  group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
  b_bot_group_.setPlanningTime(1.0);
  return false;
}

bool SkillServer::goToNamedPose(std::string pose_name, std::string robot_name)
{
  ROS_INFO_STREAM("Going to named pose " << pose_name << " with robot group " << robot_name << ".");
  // TODO: Test this.
  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);

  group_pointer->setStartStateToCurrentState();
  group_pointer->setNamedTarget(pose_name);

  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;
  
  success_plan = group_pointer->move();
  
  return true;
}

// TODO: Write this function/decide if it is needed
bool SkillServer::stop()
{
  return true;
}

moveit::planning_interface::MoveGroupInterface* SkillServer::robotNameToMoveGroup(std::string robot_name)
{
  // This function converts the name of the robot to a pointer to the member variable containing the move group
  // Returning the move group itself does not seem to work, sadly.
  if (robot_name == "a_bot") return &a_bot_group_;
  if (robot_name == "b_bot") return &b_bot_group_;
  if (robot_name == "c_bot") return &c_bot_group_;
  if (robot_name == "front_bots") return &front_bots_group_;
  if (robot_name == "all_bots") return &all_bots_group_;
}


// ----------- Internal functions

bool SkillServer::equipScrewTool(std::string robot_name, std::string screw_tool_id)
{
  // TODO: Use status of the robot instead of a general one
  if (holding_object_) 
  {
    ROS_WARN("Robot already holds an object. Cannot pick up screw tool.");
    return false;
  }
  ROS_INFO("Spawning tool.");
  spawnTool(screw_tool_id);
  
  // Plan & execute motion to in front of holder
  geometry_msgs::PoseStamped ps_approach, ps_pickup;
  ps_approach.header.frame_id = screw_tool_id + "_link";
  // ps_approach.pose.position.y = .1;
  // ps_approach.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI / 2);

  ps_approach.pose.position.y = .1;
  ps_approach.pose.position.z = .015;
  ps_approach.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI / 2);
  ROS_INFO("Moving to approach pose PTP.");
  moveToCartPosePTP(ps_approach, robot_name);

  // Open gripper the correct amount
  openGripper(robot_name);
  // TODO: Set the collision matrix to allow movement into the tool

  // Plan & execute LINEAR motion to the tool change position
  ps_pickup = ps_approach;
  ps_pickup.pose.position.y = -0.05;
  ROS_INFO("Moving to pickup pose LIN.");
  moveToCartPoseLIN(ps_pickup, robot_name);

  // Close gripper, attach the tool object to the gripper in the Planning Scene
  closeGripper(robot_name);
  // attachTool(screw_tool_id, robot_name);

  // Plan & execute LINEAR motion away from the tool change position
  ROS_INFO("Moving back to approach pose LIN.");
  moveToCartPoseLIN(ps_approach, robot_name);

  // Optional: Move back to home
  return true;
}

bool SkillServer::putBackScrewTool(std::string robot_name)
{
  ;
  // --- THE PLAN:
  // Make sure no item is held
  // Plan & execute motion to in front of the correct holder (check the member variable of the size)
  // Plan & execute LINEAR motion to the tool change position
  // Open gripper, detach the tool object from the gripper in the Planning Scene
  // MAYBE: Set the ACM in the planning scene up to allow collisions between the gripper and everything (to avoid unnecessary calculations)
  // OR: Delete the tool from the scene
  // Plan & execute LINEAR motion away from the tool change position
  // Optional: Move back to home
}

bool SkillServer::openGripper(std::string robot_name)
{
  return sendGripperCommand(robot_name, 0.085);
}

bool SkillServer::closeGripper(std::string robot_name)
{
  return sendGripperCommand(robot_name, 0.0);
}

bool SkillServer::sendGripperCommand(std::string robot_name, double opening_width)
{
  bool finished_before_timeout;
  if ((robot_name == "b_bot") || (robot_name == "c_bot"))
  {
    // Send a goal to the action
    robotiq_msgs::CModelCommandGoal goal;
    goal.position = opening_width;    // Opening width. 0 to close, 0.085 to open the gripper.
    if (robot_name == "b_bot")
    {
      b_bot_gripper_client_.sendGoal(goal);
      finished_before_timeout = b_bot_gripper_client_.waitForResult(ros::Duration(5.0));
    }
    else if (robot_name == "c_bot")
    {
      c_bot_gripper_client_.sendGoal(goal);
      finished_before_timeout = c_bot_gripper_client_.waitForResult(ros::Duration(5.0));
    }
  }
  else
  {
    ROS_WARN("The gripper you specified is not defined.");
    return false;
  }
  return finished_before_timeout;
}

// Add the screw tool as a Collision Object to the scene, so that it can be attached to the robot
bool SkillServer::spawnTool(std::string screw_tool_id)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  if (screw_tool_id == "screw_tool_m5") collision_objects[0] = screw_tool_m5;
  else if (screw_tool_id == "screw_tool_m4") collision_objects[0] = screw_tool_m4;
  else if (screw_tool_id == "screw_tool_m3") collision_objects[0] = screw_tool_m3;
  
  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface_.applyCollisionObjects(collision_objects);

  return true;
}

// Remove the tool from the scene so it does not cause unnecessary collision calculations
bool SkillServer::despawnTool(std::string screw_tool_id)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = screw_tool_id;
    collision_objects[0].operation = collision_objects[0].REMOVE;
    planning_scene_interface_.applyCollisionObjects(collision_objects);

    return true;
}

bool SkillServer::attachTool(std::string screw_tool_id, std::string robot_name)
{
  return attachDetachTool(screw_tool_id, robot_name, "attach");
}

bool SkillServer::detachTool(std::string screw_tool_id, std::string robot_name)
{
  return attachDetachTool(screw_tool_id, robot_name, "detach");
}

bool SkillServer::attachDetachTool(std::string screw_tool_id, std::string robot_name, std::string attach_or_detach)
{
  moveit_msgs::AttachedCollisionObject att_coll_object;

  if (screw_tool_id == "screw_tool_m5") att_coll_object.object = screw_tool_m5;
  else if (screw_tool_id == "screw_tool_m4") att_coll_object.object = screw_tool_m4;
  else if (screw_tool_id == "screw_tool_m3") att_coll_object.object = screw_tool_m3;
  else { ROS_WARN_STREAM("No screw tool specified to " << attach_or_detach); }

  att_coll_object.link_name = robot_name + "_robotiq_85_tip_link";

  if (attach_or_detach == "attach") att_coll_object.object.operation = att_coll_object.object.ADD;
  else if (attach_or_detach == "detach") att_coll_object.object.operation = att_coll_object.object.REMOVE;
  
  ROS_INFO_STREAM(attach_or_detach << "ing tool " << screw_tool_id);
  planning_scene_interface_.applyAttachedCollisionObject(att_coll_object);
  return true;
}


// ----------- Service definitions
bool SkillServer::goToNamedPoseCallback(o2as_skills::goToNamedPose::Request &req,
                                           o2as_skills::goToNamedPose::Response &res)
{
  res.success = goToNamedPose(req.planning_group, req.named_pose);
  return true;
}


// ----------- Action servers

// alignAction
void SkillServer::executeAlign(const o2as_skills::alignGoalConstPtr& goal)
{
  ROS_INFO("alignAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("alignAction is set as succeeded");
  alignActionServer_.setSucceeded();
}

// pickAction
void SkillServer::executePick(const o2as_skills::pickGoalConstPtr& goal)
{
  ROS_INFO("pickAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  if (goal->gripper == "a_bot")
  {;}
  else if (goal->gripper == "b_bot")
  {;}
  else if (goal->gripper == "c_bot")
  {;}
  else if (goal->gripper == "suction")
  {;}

  ROS_INFO("pickAction is set as succeeded");
  pickActionServer_.setSucceeded();
}

// placeAction
void SkillServer::executePlace(const o2as_skills::placeGoalConstPtr& goal)
{
  ROS_INFO("placeAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("placeAction is set as succeeded");
  placeActionServer_.setSucceeded();
}

// insertAction
void SkillServer::executeInsert(const o2as_skills::insertGoalConstPtr& goal)
{
  ROS_INFO("insertAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("insertAction is set as succeeded");
  insertActionServer_.setSucceeded();
}

// screwAction
void SkillServer::executeScrew(const o2as_skills::screwGoalConstPtr& goal)
{
  ROS_INFO("screwAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("screwAction is set as succeeded");
  screwActionServer_.setSucceeded();
}


// ----------- End of the class definitions

int main(int argc, char **argv)
{
  ros::init(argc, argv, "o2as_skills");
  ros::AsyncSpinner spinner(1); // Needed for MoveIt to work.
  spinner.start();

  // Create an object of class SkillServer that will take care of everything
  SkillServer o2as_skill_server;

  ROS_INFO("O2AS skill server started");

  ROS_INFO("Testing the screw tool mounting.");
  o2as_skill_server.equipScrewTool("b_bot", "screw_tool_m5");

  ROS_INFO("Going to home pose.");
  o2as_skill_server.goToNamedPose("home_b", "b_bot");

  ROS_INFO("Testing the screw tool unmounting.");
  o2as_skill_server.putBackScrewTool("b_bot");
  ROS_INFO("Done.");

  return 0;
}
