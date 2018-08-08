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
  front_bots_group_.setPlanningTime(3.0);
  front_bots_group_.setPlannerId("RRTConnectkConfigDefault");
  all_bots_group_.setPlanningTime(5.0);
  all_bots_group_.setPlannerId("RRTConnectkConfigDefault");

  get_planning_scene_client = n_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  // Get the planning scene of the movegroup
  updatePlanningScene();

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
bool SkillServer::moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait, std::string end_effector_link)
{
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);
  
  group_pointer->clearPoseTargets();
  group_pointer->setStartStateToCurrentState();
  group_pointer->setEndEffectorLink(end_effector_link);
  group_pointer->setPoseTarget(pose);

  ROS_DEBUG_STREAM("Planning motion for robot " << robot_name << " and EE link " << end_effector_link + "_tip_link.");
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
bool SkillServer::moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait, std::string end_effector_link)
{
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);

  group_pointer->clearPoseTargets();
  group_pointer->setStartStateToCurrentState();
  group_pointer->setEndEffectorLink(end_effector_link);
  
  // Plan cartesian motion
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped start_pose;
  start_pose.header.frame_id = robot_name + "_robotiq_85_tip_link";
  start_pose.pose = makePose();
  start_pose = transform_pose_now(start_pose, "world", tflistener_);
  pose = transform_pose_now(pose, "world", tflistener_);
  waypoints.push_back(start_pose.pose);
  waypoints.push_back(pose.pose);

  group_pointer->setMaxVelocityScalingFactor(0.1);  // Does this work??
  b_bot_group_.setPlanningTime(20.0);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double cartesian_success = group_pointer->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("Cartesian motion plan was %.2f%% successful.", cartesian_success * 100.0);

  // --- This was for testing the 
  // ros::Publisher pubLinTraj = n_.advertise<moveit_msgs::RobotTrajectory>("/linear_trajectory", 1);
  // ROS_INFO("Publishing trajectory.");
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // pubLinTraj.publish(trajectory);

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
      if (cartesian_success > .95) return true;
      else return false;
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
  return equipUnequipScrewTool(robot_name, screw_tool_id, "equip");
}

bool SkillServer::unequipScrewTool(std::string robot_name)
{
  return equipUnequipScrewTool(robot_name, held_screw_tool_, "unequip");
}

bool SkillServer::equipUnequipScrewTool(std::string robot_name, std::string screw_tool_id, std::string equip_or_unequip)
{
  // Sanity check on the input instruction
  bool equip = (equip_or_unequip == "equip");
  bool unequip = (equip_or_unequip == "unequip");   
  // The second comparison is not always necessary, but readability comes first.
  if ((!equip) && (!unequip))
  {
    ROS_ERROR_STREAM("Cannot read the instruction " << equip_or_unequip << ". Returning false.");
    return false;
  }

  // TODO: Use the status of the robot_name instead of a general one
  if (holding_object_)
  {
    ROS_ERROR_STREAM("Robot already holds an object. Cannot " << equip_or_unequip << " screw tool.");
    return false;
  }
  
  // Plan & execute motion to in front of holder
  geometry_msgs::PoseStamped ps_approach, ps_tool_holder;
  ps_approach.header.frame_id = screw_tool_id + "_link";

  ps_approach.pose.position.y = .1;
  ps_approach.pose.position.z = .017;
  ps_approach.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI / 2);
  ROS_INFO("Moving to screw tool approach pose PTP.");
  moveToCartPosePTP(ps_approach, robot_name);

  if (equip) {
    openGripper(robot_name);
    ROS_INFO("Spawning tool. Waiting for a while to be safe.");
    spawnTool(screw_tool_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    held_screw_tool_ = screw_tool_id;
  }

  // Disable all collisions to allow movement into the tool
  // NOTE: This could be done cleaner by disabling only gripper + tool, but it is good enough for now.
  moveit_msgs::AllowedCollisionMatrix acm_cleared, acm_original = planning_scene_.allowed_collision_matrix;
  acm_cleared = acm_original;
  for (int i = 0; i < acm_cleared.entry_values.size(); ++i)
  {
    for (int j = 0; j < acm_cleared.entry_values[j].enabled.size(); ++j)
    {
      acm_cleared.entry_values[i].enabled[j] = true;
    }
  }
  moveit_msgs::PlanningScene ps_no_collisions = planning_scene_;
  ps_no_collisions.allowed_collision_matrix = acm_cleared;
  planning_scene_interface_.applyPlanningScene(ps_no_collisions);

  // Plan & execute LINEAR motion to the tool change position
  ps_tool_holder = ps_approach;
  if (equip)        ps_tool_holder.pose.position.y = -0.03;
  else if (unequip) ps_tool_holder.pose.position.y = -0.02;  
  // The tool is deposited a bit in front of the original position. The magnet pulls it to the final pose.
  ROS_INFO("Moving to pose in tool holder LIN.");
  moveToCartPoseLIN(ps_tool_holder, robot_name);

  // Close gripper, attach the tool object to the gripper in the Planning Scene
  if (equip)
  {
    closeGripper(robot_name);
    attachTool(screw_tool_id, robot_name);
  }
  else if (unequip) 
  {
    openGripper(robot_name);
    detachTool(screw_tool_id, robot_name);
  }
  
  // Plan & execute LINEAR motion away from the tool change position
  ROS_INFO("Moving back to screw tool approach pose LIN.");
  moveToCartPoseLIN(ps_approach, robot_name);

  // Reactivate the collisions
  planning_scene_interface_.applyPlanningScene(planning_scene_);

  // Delete tool collision object only after collision reinitialization to avoid errors
  if (unequip) despawnTool(screw_tool_id);

  return true;
}

// This refreshes the class member variable of the planning scene.
bool SkillServer::updatePlanningScene()
{
  moveit_msgs::GetPlanningScene srv;
  // Request only the collision matrix
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  get_planning_scene_client.call(srv);
  if (get_planning_scene_client.call(srv))
  {
    ROS_INFO("Got planning scene from move group.");
    planning_scene_ = srv.response.scene;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to get planning scene from move group.");
    return false;
  }
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

bool SkillServer::pickScrew(std::string object_id, std::string screw_tool_id, std::string robot_name)
{
  // Retrieve the position of the object from the planning scene
  geometry_msgs::PoseStamped object_pose = makePoseStamped();
  std::vector<std::string> object_name_vector;
  object_name_vector.push_back(object_id);
  std::map< std::string, geometry_msgs::Pose > poses = planning_scene_interface_.getObjectPoses(object_name_vector);
  
  object_pose.header.frame_id = "world";
  object_pose.pose.position = poses[object_id].position;
  // We ignore the orientation of the screw (they always point down) and assign an appropriate orientation
  // TODO: How to find a good pose for the robot automatically? Launch a planning request to MoveIt?
  object_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -(145.0/180.0 *M_PI));   // Z-axis pointing up.
  
  std::string screw_tool_link = robot_name + "_" + screw_tool_id + "_tip_link";
  return pickFromAbove(object_pose, screw_tool_link, robot_name);
}

bool SkillServer::pickFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name)
{
  // Move above the screw
  target_tip_link_pose.pose.position.z += .1;
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);

  // Move onto the screw
  target_tip_link_pose.pose.position.z -= .1;
  bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name);
  if (!success) moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
  
  // Move back up a little
  target_tip_link_pose.pose.position.z += .05;
  success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name);
  if (!success) moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);  // Force the move even if LIN fails

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

  if (goal->tool_name == "screw_tool")
  {
    std::string screw_tool_id = goal->tool_name + "_m" + std::to_string(goal->screw_size);
    pickScrew(goal->item_id, screw_tool_id, goal->robot_name);
  }
  else if (goal->tool_name == "suction")
  {;} // TODO: Here is space for code from AIST.
  else // No tool being used
  {
    ;
    // TODO. The plan: 
    // - Check that the object exists in the planning scene
    // - VARIATION A:
    // -- Pass the request to the grasp planner along with the robot name
    // -- It should return a grasp to be executed via the modified MoveIt pick routine
    // - VARIATION B:
    // -- Take the center of gravity and the major axis of the object
    // -- Move the gripper to above the object, orienting the major axis along the z-axis of the tip link
    // -- Allow collisions with the object and the gripper
    // -- Move down to an appropriate gripping height or to the bottom of the container
    // -- Close the grasp, then move back up
  }

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

  ROS_INFO("Picking screw from tray 1.");
  geometry_msgs::PoseStamped ps = makePoseStamped();
  ps.header.frame_id = "set3_tray_1";
  ps.pose.position.z = .02;
  // We ignore the orientation of the screw (they always point down) and assign an appropriate orientation
  // TODO: How to find a good pose for the robot automatically? Launch a planning request to MoveIt?
  ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -(110.0/180.0 *M_PI));   // Z-axis pointing up.
  std::string link_name = "b_bot_screw_tool_m5_tip_link";
  std::string robot_name = "b_bot";

  o2as_skill_server.pickFromAbove(ps, link_name, robot_name);

  ROS_INFO("Picking screw from tray 2.");
  ps.header.frame_id = "set3_tray_2";
  ps.pose.position.y = .05;
  o2as_skill_server.pickFromAbove(ps, link_name, robot_name);

  ROS_INFO("Going to home pose again.");
  o2as_skill_server.goToNamedPose("home_b", "b_bot");

  ROS_INFO("Testing the screw tool unmounting.");
  o2as_skill_server.unequipScrewTool("b_bot");
  ROS_INFO("Done.");

  return 0;
}
