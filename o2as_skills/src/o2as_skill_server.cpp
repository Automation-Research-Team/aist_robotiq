#include "o2as_skill_server.h"

SkillServer::SkillServer() : 
                  alignActionServer_(n_, "o2as_skills/align", boost::bind(&SkillServer::executeAlign, this, _1),false),
                  pickActionServer_(n_, "o2as_skills/pick", boost::bind(&SkillServer::executePick, this, _1),false),
                  placeActionServer_(n_, "o2as_skills/place", boost::bind(&SkillServer::executePlace, this, _1),false),
                  regraspActionServer_(n_, "o2as_skills/regrasp", boost::bind(&SkillServer::executeRegrasp, this, _1),false),
                  insertActionServer_(n_, "o2as_skills/insert", boost::bind(&SkillServer::executeInsert, this, _1),false),
                  screwActionServer_(n_, "o2as_skills/screw", boost::bind(&SkillServer::executeScrew, this, _1),false),
                  changeToolActionServer_(n_, "o2as_skills/changeTool", boost::bind(&SkillServer::executeChangeTool, this, _1),false),
                  a_bot_group_("a_bot"), b_bot_group_("b_bot"), c_bot_group_("c_bot"),
                  b_bot_gripper_client_("/b_bot_gripper/gripper_action_controller", true),
                  c_bot_gripper_client_("/c_bot_gripper/gripper_action_controller", true),
                  fastening_tool_client("/o2as_fastening_tools/fastener_gripper_control_action", true)
{ 
  // Topics to publish
  pubMarker_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Services to advertise
  goToNamedPoseService_ = n_.advertiseService("o2as_skills/goToNamedPose", &SkillServer::goToNamedPoseCallback,
                                        this);
  publishMarkerService_ = n_.advertiseService("o2as_skills/publishMarker", &SkillServer::publishMarkerCallback,
                                        this);
  toggleCollisionsService_ = n_.advertiseService("o2as_skills/toggleCollisions", &SkillServer::toggleCollisionsCallback,
                                        this);

  // Services to subscribe to
  sendScriptToURClient_ = n_.serviceClient<o2as_msgs::sendScriptToUR>("o2as_skills/sendScriptToUR");
  PrecisionGripperClient_ = n_.serviceClient<o2as_msgs::PrecisionGripperCommand>("precision_gripper_command");

  // Actions we serve
  alignActionServer_.start();
  pickActionServer_.start();
  placeActionServer_.start();
  regraspActionServer_.start();
  insertActionServer_.start();
  screwActionServer_.start();
  changeToolActionServer_.start();

  // Action clients
  // ROS_INFO("Waiting for action servers to start.");
  // // a_bot_gripper_client.waitForServer(); 
  // b_bot_gripper_client_.waitForServer();
  // c_bot_gripper_client_.waitForServer();
  // ROS_INFO("Action servers started.");

  // Set up MoveGroups
  a_bot_group_.setPlanningTime(PLANNING_TIME);
  a_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  a_bot_group_.setEndEffectorLink("a_bot_gripper_tip_link");
  a_bot_group_.setNumPlanningAttempts(5);
  b_bot_group_.setPlanningTime(PLANNING_TIME);
  b_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  b_bot_group_.setEndEffectorLink("b_bot_robotiq_85_tip_link");
  b_bot_group_.setNumPlanningAttempts(5);
  c_bot_group_.setPlanningTime(PLANNING_TIME);
  c_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  c_bot_group_.setEndEffectorLink("c_bot_robotiq_85_tip_link");
  c_bot_group_.setNumPlanningAttempts(5);
  // front_bots_group_.setPlanningTime(PLANNING_TIME);
  // front_bots_group_.setPlannerId("RRTConnectkConfigDefault");
  // all_bots_group_.setPlanningTime(PLANNING_TIME);
  // all_bots_group_.setPlannerId("RRTConnectkConfigDefault");

  get_planning_scene_client = n_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  // Get the planning scene of the movegroup
  updatePlanningScene();

  // --- Define the screw tools.
  
  // Define one of the tools as a collision object
  //M6 tool
  screw_tool_m6.header.frame_id = "screw_tool_m6_link";
  screw_tool_m6.id = "screw_tool_m6";

  screw_tool_m6.primitives.resize(3);
  screw_tool_m6.primitive_poses.resize(3);
  // The bit cushion and motor
  screw_tool_m6.primitives[0].type = screw_tool_m6.primitives[0].BOX;
  screw_tool_m6.primitives[0].dimensions.resize(3);
  screw_tool_m6.primitives[0].dimensions[0] = 0.026;
  screw_tool_m6.primitives[0].dimensions[1] = 0.04;
  screw_tool_m6.primitives[0].dimensions[2] = 0.055;
  screw_tool_m6.primitive_poses[0].position.x = 0;
  screw_tool_m6.primitive_poses[0].position.y = -0.009;
  screw_tool_m6.primitive_poses[0].position.z = 0.0275;

  // The "shaft" + suction attachment
  screw_tool_m6.primitives[1].type = screw_tool_m6.primitives[1].BOX;
  screw_tool_m6.primitives[1].dimensions.resize(3);
  screw_tool_m6.primitives[1].dimensions[0] = 0.02;
  screw_tool_m6.primitives[1].dimensions[1] = 0.03;
  screw_tool_m6.primitives[1].dimensions[2] = 0.091;
  screw_tool_m6.primitive_poses[1].position.x = 0;
  screw_tool_m6.primitive_poses[1].position.y = -0.0055;  // 21 mm distance from axis
  screw_tool_m6.primitive_poses[1].position.z = -0.041;

  // The cylinder representing the tip
  screw_tool_m6.primitives[2].type = screw_tool_m6.primitives[2].CYLINDER;
  screw_tool_m6.primitives[2].dimensions.resize(2);
  screw_tool_m6.primitives[2].dimensions[0] = 0.019;    // Cylinder height
  screw_tool_m6.primitives[2].dimensions[1] = 0.005;   // Cylinder radius
  screw_tool_m6.primitive_poses[2].position.x = 0;
  screw_tool_m6.primitive_poses[2].position.y = 0;  // 21 mm distance from axis
  screw_tool_m6.primitive_poses[2].position.z = -0.096;
  screw_tool_m6.operation = screw_tool_m6.ADD;

  // The tool tip
  screw_tool_m6.frame_poses.resize(1);
  screw_tool_m6.frame_names.resize(1);
  screw_tool_m6.frame_poses[0].position.z = -.11;
  screw_tool_m6.frame_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 90.0/180.0 *M_PI, -M_PI/2);
  screw_tool_m6.frame_names[0] = "screw_tool_m6_tip";
  

  //M4 tool
  screw_tool_m4.header.frame_id = "screw_tool_m4_link";
  screw_tool_m4.id = "screw_tool_m4";

  screw_tool_m4.primitives.resize(3);
  screw_tool_m4.primitive_poses.resize(3);
  // The bit cushion and motor
  screw_tool_m4.primitives[0].type = screw_tool_m4.primitives[0].BOX;
  screw_tool_m4.primitives[0].dimensions.resize(3);
  screw_tool_m4.primitives[0].dimensions[0] = 0.026;
  screw_tool_m4.primitives[0].dimensions[1] = 0.04;
  screw_tool_m4.primitives[0].dimensions[2] = 0.055;
  screw_tool_m4.primitive_poses[0].position.x = 0;
  screw_tool_m4.primitive_poses[0].position.y = -0.009;
  screw_tool_m4.primitive_poses[0].position.z = 0.0275;

  // The "shaft" + suction attachment
  screw_tool_m4.primitives[1].type = screw_tool_m4.primitives[1].BOX;
  screw_tool_m4.primitives[1].dimensions.resize(3);
  screw_tool_m4.primitives[1].dimensions[0] = 0.02;
  screw_tool_m4.primitives[1].dimensions[1] = 0.03;
  screw_tool_m4.primitives[1].dimensions[2] = 0.08;
  screw_tool_m4.primitive_poses[1].position.x = 0;
  screw_tool_m4.primitive_poses[1].position.y = -0.0055;  // 21 mm distance from axis
  screw_tool_m4.primitive_poses[1].position.z = -0.04;

  // The cylinder representing the tip
  screw_tool_m4.primitives[2].type = screw_tool_m4.primitives[2].CYLINDER;
  screw_tool_m4.primitives[2].dimensions.resize(2);
  screw_tool_m4.primitives[2].dimensions[0] = 0.038;    // Cylinder height
  screw_tool_m4.primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
  screw_tool_m4.primitive_poses[2].position.x = 0;
  screw_tool_m4.primitive_poses[2].position.y = 0;  // 21 mm distance from axis
  screw_tool_m4.primitive_poses[2].position.z = -0.099;
  screw_tool_m4.operation = screw_tool_m4.ADD;

  // The tool tip
  screw_tool_m4.frame_poses.resize(1);
  screw_tool_m4.frame_names.resize(1);
  screw_tool_m4.frame_poses[0].position.z = -.12;
  screw_tool_m4.frame_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 90.0/180.0 *M_PI, -M_PI/2);
  screw_tool_m4.frame_names[0] = "screw_tool_m4_tip";


  //M3 tool
  screw_tool_m3.header.frame_id = "screw_tool_m3_link";
  screw_tool_m3.id = "screw_tool_m3";

  screw_tool_m3.primitives.resize(3);
  screw_tool_m3.primitive_poses.resize(3);
  // The bit cushion and motor
  screw_tool_m3.primitives[0].type = screw_tool_m3.primitives[0].BOX;
  screw_tool_m3.primitives[0].dimensions.resize(3);
  screw_tool_m3.primitives[0].dimensions[0] = 0.026;
  screw_tool_m3.primitives[0].dimensions[1] = 0.04;
  screw_tool_m3.primitives[0].dimensions[2] = 0.055;
  screw_tool_m3.primitive_poses[0].position.x = 0;
  screw_tool_m3.primitive_poses[0].position.y = -0.009;
  screw_tool_m3.primitive_poses[0].position.z = 0.0275;

  // The "shaft" + suction attachment
  screw_tool_m3.primitives[1].type = screw_tool_m3.primitives[1].BOX;
  screw_tool_m3.primitives[1].dimensions.resize(3);
  screw_tool_m3.primitives[1].dimensions[0] = 0.02;
  screw_tool_m3.primitives[1].dimensions[1] = 0.03;
  screw_tool_m3.primitives[1].dimensions[2] = 0.08;
  screw_tool_m3.primitive_poses[1].position.x = 0;
  screw_tool_m3.primitive_poses[1].position.y = -0.0055;  // 21 mm distance from axis
  screw_tool_m3.primitive_poses[1].position.z = -0.04;

  // The cylinder representing the tip
  screw_tool_m3.primitives[2].type = screw_tool_m3.primitives[2].CYLINDER;
  screw_tool_m3.primitives[2].dimensions.resize(2);
  screw_tool_m3.primitives[2].dimensions[0] = 0.018;    // Cylinder height
  screw_tool_m3.primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
  screw_tool_m3.primitive_poses[2].position.x = 0;
  screw_tool_m3.primitive_poses[2].position.y = 0;  // 21 mm distance from axis
  screw_tool_m3.primitive_poses[2].position.z = -0.089;
  screw_tool_m3.operation = screw_tool_m3.ADD;

  // The tool tip
  screw_tool_m3.frame_poses.resize(1);
  screw_tool_m3.frame_names.resize(1);
  screw_tool_m3.frame_poses[0].position.z = -.11;
  screw_tool_m3.frame_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 90.0/180.0 *M_PI, -M_PI/2);
  screw_tool_m3.frame_names[0] = "screw_tool_m3_tip";


  // Initialize robot statuses
  o2as_msgs::RobotStatus r1, r2, r3;
  robot_statuses_["a_bot"] = r1;
  robot_statuses_["b_bot"] = r2;
  robot_statuses_["c_bot"] = r3;
  
  n_.getParam("use_real_robot", use_real_robot_);
  ROS_INFO_STREAM((use_real_robot_ ? "Using real robot!" : "Using simulated robot."));
  if (use_real_robot_)
  {
    ROS_INFO_STREAM("Using real robot!");
  }
  else
  {
    ROS_INFO_STREAM("Using simulated robot.");
  }
}

bool SkillServer::moveToJointPose(std::vector<double> joint_positions, std::string robot_name, bool wait, double velocity_scaling_factor, bool use_UR_script)
{
  if (joint_positions.size() != 6)
  {
    ROS_ERROR_STREAM("Size of joint positions in moveToJointPose is not correct! Expected 6, got " << joint_positions.size());
    return false;
  }
  if (use_UR_script)
  {
    o2as_msgs::sendScriptToUR UR_srv;
    UR_srv.request.program_id = "movej";
    UR_srv.request.robot_name = robot_name;  
    UR_srv.request.joint_positions = joint_positions;
    UR_srv.request.velocity = velocity_scaling_factor;
    if (UR_srv.response.success == true)
    {
      ROS_INFO("Successfully called the URScript client to move joints.");
      return waitForURProgram("/" + robot_name +"_controller");
    }
    else
    {
      ROS_ERROR("Could not move joints with URscript.");
      return false;
    }
  }

  moveit::planning_interface::MoveGroupInterface* group_pointer = robotNameToMoveGroup(robot_name);;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success, motion_done;
  group_pointer->setJointValueTarget(joint_positions);
    
  success = (group_pointer->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    motion_done = group_pointer->execute(my_plan);
    return (motion_done == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  else
  {
    ROS_ERROR("Could not plan to before_tool_pickup joint state. Abort!");
    return false;
  }
}

// This works only for a single robot.
bool SkillServer::moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait, std::string end_effector_link, double velocity_scaling_factor)
{
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);
  
  group_pointer->clearPoseTargets();
  group_pointer->setStartStateToCurrentState();
  group_pointer->setMaxVelocityScalingFactor(velocity_scaling_factor);  // TODO: Check if this works
  if (end_effector_link == "")  // Define default end effector link explicitly
  {
    if (robot_name == "a_bot") group_pointer->setEndEffectorLink("a_bot_gripper_tip_link");
    else group_pointer->setEndEffectorLink(robot_name + "_robotiq_85_tip_link");
  }
  else group_pointer->setEndEffectorLink(end_effector_link);
  group_pointer->setPoseTarget(pose);

  ROS_INFO_STREAM("Planning motion for robot " << robot_name << " and EE link " << end_effector_link + "_tip_link, to pose:");
  ROS_INFO_STREAM(pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z);
  success_plan = group_pointer->plan(myplan);
  if (success_plan) 
  {
    if (wait) motion_done = group_pointer->execute(myplan);
    else motion_done = group_pointer->asyncExecute(myplan);
    if (motion_done) {
      group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
      return true;
    }
  }
  ROS_WARN("Failed to perform motion.");
  group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
  return false;
}

// This works only for a single robot.
bool SkillServer::moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait, std::string end_effector_link, double velocity_scaling_factor)
{
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);

  group_pointer->clearPoseTargets();
  group_pointer->setPoseReferenceFrame("world");
  group_pointer->setStartStateToCurrentState();
  group_pointer->setEndEffectorLink(end_effector_link);
  
  // Plan cartesian motion
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped start_pose, end_pose;
  if (end_effector_link == "") {
    if (robot_name == "a_bot") start_pose.header.frame_id = "a_bot_gripper_tip_link";
    else start_pose.header.frame_id = robot_name + "_robotiq_85_tip_link";
  }
  else start_pose.header.frame_id = end_effector_link;
  start_pose.pose = makePose();
  start_pose = transform_pose_now(start_pose, "world", tflistener_);
  end_pose = transform_pose_now(pose, "world", tflistener_);
  waypoints.push_back(start_pose.pose);
  waypoints.push_back(end_pose.pose);

  group_pointer->setMaxVelocityScalingFactor(velocity_scaling_factor); // This doesn't work for linear paths: https://answers.ros.org/question/288989/moveit-velocity-scaling-for-cartesian-path/
  b_bot_group_.setPlanningTime(LIN_PLANNING_TIME);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  ros::Time start_time = ros::Time::now();
  double cartesian_success = group_pointer->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ros::Duration d = ros::Time::now() - start_time;
  ROS_INFO_STREAM("Cartesian motion plan took " << d.toSec() << " s and was " << cartesian_success * 100.0 << "% successful.");

  // Scale the trajectory. This is workaround to setting the VelocityScalingFactor. Copied from k-okada
  if (cartesian_success > 0.95)
  {
    moveit_msgs::RobotTrajectory scaled_trajectory = moveit_msgs::RobotTrajectory(trajectory);
    // Scaling (https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4)
    // The trajectory needs to be modified so it will include velocities as well.
    // First: create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(group_pointer->getRobotModel(), group_pointer->getName());
    // Second: get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*group_pointer->getCurrentState(), scaled_trajectory);
    // Third: create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth: compute computeTimeStamps
    bool success =
        iptp.computeTimeStamps(rt, velocity_scaling_factor, velocity_scaling_factor);  // The second value is actually acceleration
    ROS_INFO_STREAM("Computing time stamps for iptp scaling with factor " << velocity_scaling_factor << " " << success ? "SUCCEEDED" : "FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(scaled_trajectory);
    // Fill in move_group_
    myplan.trajectory_ = scaled_trajectory;
  
    if (true) 
    {
      if (wait) motion_done = group_pointer->execute(myplan);
      else motion_done = group_pointer->asyncExecute(myplan);
      if (motion_done) 
      {
        group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
        group_pointer->setPlanningTime(1.0);
        if (cartesian_success > .95) return true;
        else return false;
      }
    }
  }
  else
  {
    if (use_real_robot_)
    {
      ROS_WARN("MoveIt failed to do linear plan. Trying move_l with the UR.");
      ros::Duration(2).sleep();
      o2as_msgs::sendScriptToUR UR_srv;
      UR_srv.request.program_id = "lin_move";
      UR_srv.request.robot_name = robot_name;  
      UR_srv.request.target_pose = transformTargetPoseFromTipLinkToEE(pose, robot_name, end_effector_link, tflistener_);
      publishMarker(transformTargetPoseFromTipLinkToEE(pose, robot_name, end_effector_link, tflistener_), "pose");
      UR_srv.request.velocity = .05;
      sendScriptToURClient_.call(UR_srv);
      if (UR_srv.response.success == true)
      {
        ROS_INFO("Successfully called the URScript client to do linear motion to approach pose.");
        waitForURProgram("/" + robot_name +"_controller");
        return true;
      }
      else
      {
        ROS_ERROR("Could not go to approach pose. Aborting tool pickup.");
        planning_scene_interface_.applyPlanningScene(planning_scene_);
        return false;
      }
    }
    ROS_ERROR_STREAM("Cartesian motion plan failed.");
    group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
    group_pointer->setPlanningTime(1.0);
    return false;
  }
}

bool SkillServer::goToNamedPose(std::string pose_name, std::string robot_name)
{
  ROS_INFO_STREAM("Going to named pose " << pose_name << " with robot group " << robot_name << ".");
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
  // if (robot_name == "front_bots") return &front_bots_group_;
  // if (robot_name == "all_bots") return &all_bots_group_;
}

std::string SkillServer::getEELink(std::string robot_name)
{
  std::string ee_link_name;
  moveit::planning_interface::MoveGroupInterface* group_pointer = robotNameToMoveGroup(robot_name);
  ee_link_name = group_pointer->getEndEffectorLink();
  if (ee_link_name == "")
  {
    ROS_ERROR("Requested end effector was returned empty!");
  }
  return ee_link_name;
}


// ----------- Internal functions

bool SkillServer::equipScrewTool(std::string robot_name, std::string screw_tool_id)
{
  ROS_INFO_STREAM("Equipping screw tool " << screw_tool_id);
  return equipUnequipScrewTool(robot_name, screw_tool_id, "equip");
}

bool SkillServer::unequipScrewTool(std::string robot_name)
{
  ROS_INFO_STREAM("Unequipping screw tool " << held_screw_tool_);
  return equipUnequipScrewTool(robot_name, held_screw_tool_, "unequip");
}

bool SkillServer::equipUnequipScrewTool(std::string robot_name, std::string screw_tool_id, std::string equip_or_unequip)
{
  // Sanity check on the input instruction
  bool equip = (equip_or_unequip == "equip");
  bool unequip = (equip_or_unequip == "unequip");
  double lin_speed = 0.01;
  // The second comparison is not always necessary, but readability comes first.
  if ((!equip) && (!unequip))
  {
    ROS_ERROR_STREAM("Cannot read the instruction " << equip_or_unequip << ". Returning false.");
    return false;
  }

  if ((robot_statuses_[robot_name].carrying_object == true))
  {
    ROS_ERROR_STREAM("Robot holds an object. Cannot " << equip_or_unequip << " screw tool.");
    return false;
  }
  if ( (robot_statuses_[robot_name].carrying_tool == true) && (equip))
  {
    ROS_ERROR_STREAM("Robot already holds a tool. Cannot equip another.");
    return false;
  }
  if ( (robot_statuses_[robot_name].carrying_tool == false) && (unequip))
  {
    ROS_ERROR_STREAM("Robot is not holding a tool. Cannot unequip any.");
    return false;
  }
  
  ROS_INFO("Going to before_tool_pickup pose.");
  // STEP 0:
  moveit::planning_interface::MoveGroupInterface* group_pointer;
  std::vector<double> joint_group_positions_1, joint_group_positions_2, joint_group_positions_3;

  if (robot_name == "b_bot")
  {
    // Go to two joint poses so the movement to the holder is not too messy and the gripper is in the correct orientation
    group_pointer = robotNameToMoveGroup(robot_name);
    moveit::core::RobotStatePtr current_state = group_pointer->getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = 
                group_pointer->getCurrentState()->getJointModelGroup(robot_name);
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_1);
    ROS_INFO_STREAM("Current joint state: " << joint_group_positions_1[0] << ", " << joint_group_positions_1[1] << "...");

    current_state = group_pointer->getCurrentState();
    joint_model_group = group_pointer->getCurrentState()->getJointModelGroup(robot_name);
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_1);
    ROS_INFO_STREAM("Current joint state: " << joint_group_positions_1[0] << ", " << joint_group_positions_1[1] << "...");

    current_state = group_pointer->getCurrentState();
    joint_model_group = group_pointer->getCurrentState()->getJointModelGroup(robot_name);
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_1);
    ROS_INFO_STREAM("Current joint state: " << joint_group_positions_1[0] << ", " << joint_group_positions_1[1] << "...");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success, motion_done;
    if (joint_group_positions_1[0] > -0.2)
    {
      // This position puts the gripper up high, in preparation of going to the holder
      joint_group_positions_1[0] = 1.5722;
      joint_group_positions_1[1] = -2.6696;
      joint_group_positions_1[2] = 1.7915;
      joint_group_positions_1[3] = 0.872;
      joint_group_positions_1[4] = 1.5723;
      joint_group_positions_1[5] = -3.1413;
      group_pointer->setJointValueTarget(joint_group_positions_1);
      
      if (!moveToJointPose(joint_group_positions_1, robot_name, true, 1.0, false))
      {
        ROS_ERROR("Could not plan to before_tool_pickup joint state. Abort!");
        return false;
      }
      joint_group_positions_2 = joint_group_positions_1;
      joint_group_positions_2[0] = -0.5722;
      joint_group_positions_2[1] = -2.6696;
      joint_group_positions_2[2] = 1.7915;
      joint_group_positions_2[3] = 0.872;
      joint_group_positions_2[4] = -0.5723;
      joint_group_positions_2[5] = -3.1413;
      group_pointer->setJointValueTarget(joint_group_positions_2);
      group_pointer->move();
    }
    else
      ROS_INFO("Skipping intermediate high approach pose with b_bot");

    // This position is in front of the tool holder
    joint_group_positions_3 = joint_group_positions_1;
    joint_group_positions_3[0] = -0.561;
    joint_group_positions_3[1] = -0.848;
    joint_group_positions_3[2] = 1.689;
    joint_group_positions_3[3] = -0.841;
    joint_group_positions_3[4] = -0.548;
    joint_group_positions_3[5] = -3.142;
    group_pointer->setJointValueTarget(joint_group_positions_3);
    group_pointer->move();
  }

  // Set up poses
  geometry_msgs::PoseStamped ps_approach, ps_tool_holder, ps_move_away, ps_high_up, ps_end;
  ps_approach.header.frame_id = screw_tool_id + "_helper_link";

  if (robot_name == "b_bot")
  {
    ps_approach.pose.position.x = -.06;
    ps_approach.pose.position.z = .017;
    ps_approach.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    ps_move_away = ps_approach;

    // The tool is deposited a bit in front of the original position. The robot pushes it to the final pose after placing it.
    ps_tool_holder = ps_approach;
    if (equip)        ps_tool_holder.pose.position.x = 0.030;
    else if (unequip) ps_tool_holder.pose.position.x = 0.029;  

    ps_high_up = ps_approach;
    // ps_high_up.pose.position.z +=.5;
    // ps_high_up.pose.position.x -=.1;

    ps_end = ps_high_up;
    // ps_end.pose.position.x +=.3;
    // ps_end.pose.position.y +=.4;
    // ps_end.pose.position.z -=.1;
  }
  else if (robot_name == "c_bot")
  {
    ps_approach.pose.position.x = -.04;
    ps_approach.pose.position.y = -.002;  // ATTENTION: MAGIC NUMBER!
    ps_approach.pose.position.z = .07;
    ps_approach.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, M_PI/2, 0);

    ps_tool_holder = ps_approach;
    ps_tool_holder.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, M_PI/4, 0);
    if (equip)        ps_tool_holder.pose.position.x = 0.025;
    else if (unequip) ps_tool_holder.pose.position.x = 0.024;  
    ps_tool_holder.pose.position.z = .003;
    
    ps_move_away = ps_tool_holder;
    ps_move_away.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, M_PI/2, 0);
    ps_move_away.pose.position.x = -.12;
    ps_move_away.pose.position.z = .06;

    ps_high_up = ps_move_away;
    // ps_high_up.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, M_PI*3/4, 0);
    // ps_high_up.pose.position.z +=.25;
    // ps_high_up.pose.position.y +=.05;
  }

  if (equip) {
    openGripper(robot_name);
    ROS_INFO("Spawning tool.");
    spawnTool(screw_tool_id);
    held_screw_tool_ = screw_tool_id;
  }

  // Disable all collisions to allow movement into the tool
  // NOTE: This could be done cleaner by disabling only gripper + tool, but it is good enough for now.
  updatePlanningScene();
  ROS_INFO("Disabling all collisions Updating collision matrix.");
  collision_detection::AllowedCollisionMatrix acm_no_collisions(planning_scene_.allowed_collision_matrix),
                                              acm_original(planning_scene_.allowed_collision_matrix);
  acm_no_collisions.setEntry(screw_tool_id, true);   // Allow collisions with screw tool during pickup,
  acm_original.setEntry(screw_tool_id, false); // but not afterwards.
  std::vector<std::string> entries;
  acm_no_collisions.getAllEntryNames(entries);
  for (auto i : entries)
  {
    acm_no_collisions.setEntry(i, true);
  }
  moveit_msgs::PlanningScene ps_no_collisions = planning_scene_;
  acm_no_collisions.getMessage(ps_no_collisions.allowed_collision_matrix);
  planning_scene_interface_.applyPlanningScene(ps_no_collisions);


  ROS_INFO("Moving to screw tool approach pose LIN.");
  bool preparation_succeeded = moveToCartPoseLIN(ps_approach, robot_name, true, robot_name + "_robotiq_85_tip_link", 1.0);
  if (!preparation_succeeded)
  {
    ROS_ERROR("Could not go to approach pose. Aborting tool pickup.");
    planning_scene_interface_.applyPlanningScene(planning_scene_);
    return false;
  }

  // Plan & execute linear motion to the tool change position
  ROS_INFO("Moving to pose in tool holder LIN.");
  bool moved_to_tool_holder = true;

  
  o2as_msgs::sendScriptToUR UR_srv;
  geometry_msgs::Point t_rel;
  if (equip)        lin_speed = 0.3;
  else if (unequip) lin_speed = 0.08;  
  UR_srv.request.program_id = "lin_move_rel";
  UR_srv.request.robot_name = robot_name;  
  if ( (use_real_robot_) && (robot_name == "b_bot") )
  {
    ros::Duration(.3).sleep();
    UR_srv.request.velocity = lin_speed;
    t_rel.z = (ps_tool_holder.pose.position.x - ps_approach.pose.position.x);
    UR_srv.request.relative_translation = t_rel;
    sendScriptToURClient_.call(UR_srv);
    if (UR_srv.response.success == true)
    {
      ROS_INFO("Successfully called the URScript client to perform a linear movement forward.");
      waitForURProgram("/" + robot_name +"_controller");
    }
    else
    {
      ROS_WARN("Could not call the URScript client to perform a linear movement forward.");
    }
  }
  else 
  {
    moved_to_tool_holder = moveToCartPoseLIN(ps_tool_holder, robot_name, true, "", lin_speed);
  }
  
  if (!moved_to_tool_holder) 
  {
    ROS_ERROR("Was not able to move to tool holder. ABORTING!");
    return false;
  }

  // Close gripper, attach the tool object to the gripper in the Planning Scene.
  // Its collision with the parent link is set to allowed in the original planning scene.
  if (equip)
  {
    closeGripper(robot_name);
    attachTool(screw_tool_id, robot_name);
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_tip_link", true);  // For afterwards
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_left_finger_tip_link", true);  // For afterwards
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_left_inner_knuckle_link", true);  // For afterwards
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_right_finger_tip_link", true);  // For afterwards
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_right_inner_knuckle_link", true);  // For afterwards
    
    acm_no_collisions.setEntry(screw_tool_id, true);      // To allow collisions now
    planning_scene_interface_.applyPlanningScene(ps_no_collisions);
    
    robot_statuses_[robot_name].carrying_tool = true;
    robot_statuses_[robot_name].held_tool_id = screw_tool_id;
  }
  else if (unequip) 
  {
    openGripper(robot_name);
    detachTool(screw_tool_id, robot_name);
    held_screw_tool_ = "";
    acm_original.removeEntry(screw_tool_id);
    robot_statuses_[robot_name].carrying_tool = false;
    robot_statuses_[robot_name].held_tool_id = "";
  }
  acm_original.getMessage(planning_scene_.allowed_collision_matrix);
  ros::Duration(.5).sleep();
  
  // Plan & execute linear motion away from the tool change position
  ROS_INFO("Moving back to screw tool approach pose LIN.");
  if (equip)        lin_speed = 0.08;
  else if (unequip) lin_speed = 0.3;

  if ( (use_real_robot_) && (robot_name == "b_bot") )
  {
    ros::Duration(.3).sleep();
    UR_srv.request.velocity = .05;
    t_rel.z = -(ps_tool_holder.pose.position.x - ps_approach.pose.position.x);
    UR_srv.request.relative_translation = t_rel;
    sendScriptToURClient_.call(UR_srv);
    if (UR_srv.response.success == true)
    {
      ROS_INFO("Successfully called the URScript client to perform a linear movement backward.");
      waitForURProgram("/" + robot_name +"_controller");
    }
    else
    {
      ROS_WARN("Could not call the URScript client to perform a linear movement backward.");
    }
  }
  else 
    moveToCartPoseLIN(ps_move_away, robot_name, true, "", lin_speed);
  
  // Reactivate the collisions, with the updated entry about the tool
  planning_scene_interface_.applyPlanningScene(planning_scene_);

  ROS_INFO("Moving higher up to facilitate later movements.");
  moveToCartPoseLIN(ps_high_up, robot_name);
  
  if (robot_name == "b_bot")
  {
    ROS_INFO("Going to joint pose 3.");
    group_pointer->setJointValueTarget(joint_group_positions_3);
    group_pointer->move();
    ROS_INFO("Going to joint pose 2.");
    group_pointer->setJointValueTarget(joint_group_positions_2);
    group_pointer->move();
    ROS_INFO("Going to joint pose 1.");
    group_pointer->setJointValueTarget(joint_group_positions_1);
    group_pointer->move();
    ROS_INFO("Done with joint poses, and with life.");
  }
  
  // Delete tool collision object only after collision reinitialization to avoid errors
  if (unequip) despawnTool(screw_tool_id);

  if (unequip)
  {
    goToNamedPose("home", robot_name);
  }
  else
  {
    if (robot_name == "b_bot")
    {
      // goToNamedPose("screw_ready", robot_name);
    }
    else if (robot_name == "c_bot")
    {
      goToNamedPose("screw_ready", robot_name);
    }
  }

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


bool SkillServer::toggleCollisions(bool collisions_on)
{
  if (!collisions_on)
  {
    updatePlanningScene();
    ROS_INFO("Disabling all collisions.");
    collision_detection::AllowedCollisionMatrix acm_no_collisions(planning_scene_.allowed_collision_matrix),
                                                acm_original(planning_scene_.allowed_collision_matrix);
    std::vector<std::string> entries;
    acm_no_collisions.getAllEntryNames(entries);
    for (auto i : entries)
    {
      acm_no_collisions.setEntry(i, true);
    }
    moveit_msgs::PlanningScene ps_no_collisions = planning_scene_;
    acm_no_collisions.getMessage(ps_no_collisions.allowed_collision_matrix);
    planning_scene_interface_.applyPlanningScene(ps_no_collisions);
  }
  else
  {
    ROS_INFO("Reenabling collisions with the scene as remembered.");
    planning_scene_interface_.applyPlanningScene(planning_scene_);
  }
  return true;
}


bool SkillServer::openGripper(std::string robot_name, std::string gripper_name)
{
  return sendGripperCommand(robot_name, 0.085, gripper_name);
}

bool SkillServer::closeGripper(std::string robot_name, std::string gripper_name)
{
  return sendGripperCommand(robot_name, 0.0, gripper_name);
}

bool SkillServer::sendGripperCommand(std::string robot_name, double opening_width, std::string gripper_name)
{
  bool finished_before_timeout;
  ROS_INFO_STREAM("Sending opening_width " << opening_width << " to gripper of: " << robot_name);
  if ((robot_name == "a_bot"))
  {
    ROS_DEBUG_STREAM("Gripper: " << gripper_name);
    o2as_msgs::PrecisionGripperCommand srv;
    
    if (gripper_name == "")
    {
      ROS_WARN("No gripper was defined for a_bot! Using outer_gripper by default.");
      gripper_name = "outer_gripper";
    }
    
    if (gripper_name == "outer_gripper")
    {
      if (opening_width < 0.01) {srv.request.close_outer_gripper_fully = true;}
      else if (opening_width > 0.05) {srv.request.open_outer_gripper_fully = true;}
    }
    else if (gripper_name == "inner_gripper")
    {
      if (opening_width < 0.01) {srv.request.close_inner_gripper_fully = true;}
      else if (opening_width > 0.05) {srv.request.open_inner_gripper_fully = true;}
    }

    PrecisionGripperClient_.call(srv);
    if (srv.response.success == true)
    {
      ROS_DEBUG("Successfully sent the precision gripper command.");
      // srv.request.stop = true;
      // PrecisionGripperClient_.call(srv);
    }
    else
      ROS_ERROR("Could not send the precision gripper command.");
  }
  else if ((robot_name == "b_bot") || (robot_name == "c_bot"))
  {
    // Send a goal to the action
    robotiq_msgs::CModelCommandGoal goal;
    robotiq_msgs::CModelCommandResultConstPtr result;

    goal.position = opening_width;    // Opening width. 0 to close, 0.085 to open the gripper.
    goal.velocity = 0.1;              // From 0.013 to 0.1
    goal.force = 100;                 // From 40 to 100
    if (robot_name == "b_bot")
    {
      b_bot_gripper_client_.sendGoal(goal);
      ros::Duration(0.5).sleep();
      finished_before_timeout = b_bot_gripper_client_.waitForResult(ros::Duration(2.0));
      result = b_bot_gripper_client_.getResult();
    }
    else if (robot_name == "c_bot")
    {
      c_bot_gripper_client_.sendGoal(goal);
      ros::Duration(0.5).sleep();
      finished_before_timeout = c_bot_gripper_client_.waitForResult(ros::Duration(4.0));
      result = c_bot_gripper_client_.getResult();
    }
    ROS_DEBUG_STREAM("Action " << (finished_before_timeout ? "returned" : "did not return before timeout") <<", with result: " << result->reached_goal);
  }
  else
  {
    ROS_ERROR("The specified gripper is not defined!");
    return false;
  }
  ROS_DEBUG("Returning from gripper command.");
  return finished_before_timeout;
}

bool SkillServer::sendFasteningToolCommand(std::string fastening_tool_name, std::string direction, bool wait, double duration, int speed)
{
  // Send a goal to the action
  o2as_msgs::FastenerGripperControlGoal goal;
  o2as_msgs::FastenerGripperControlResultConstPtr result;

  goal.fastening_tool_name = fastening_tool_name;
  goal.direction = direction;
  goal.duration = duration;
  goal.speed = speed;
  fastening_tool_client.sendGoal(goal);
  ros::Duration(0.5).sleep();
  bool finished_before_timeout = false;
  if (wait)
  {
    finished_before_timeout = fastening_tool_client.waitForResult(ros::Duration(10.0));
    result = fastening_tool_client.getResult();
    ROS_DEBUG_STREAM("Action " << (finished_before_timeout ? "returned" : "did not return before timeout") <<", with result: " << result->control_result);
    return result->control_result;
  }
  else
    return true;
  result = fastening_tool_client.getResult();
  ROS_DEBUG("Returning from motor command.");
  return result->control_result;
}

// Add the screw tool as a Collision Object to the scene, so that it can be attached to the robot
bool SkillServer::spawnTool(std::string screw_tool_id)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  if (screw_tool_id == "screw_tool_m6") collision_objects[0] = screw_tool_m6;
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

  if (screw_tool_id == "screw_tool_m6") att_coll_object.object = screw_tool_m6;
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

bool SkillServer::goFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, double velocity_scaling_factor)
{
  ROS_DEBUG_STREAM("Received goFromAbove command.");

  // Move above the target pose
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Moving above target.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);

  // Move down to the target pose
  target_tip_link_pose.pose.position.z -= .1;
  ROS_INFO_STREAM("Moving down to target.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name, velocity_scaling_factor);
  if (!success) 
  {
    ROS_INFO_STREAM("Linear motion plan to target place pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    success = moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, velocity_scaling_factor);  // Force the move even if LIN fails
  }
  return success;
}

bool SkillServer::placeFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name)
{
  publishMarker(target_tip_link_pose, "place_pose");
  ROS_DEBUG_STREAM("Received placeFromAbove command.");

  // Move above the target pose
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Moving above object target place.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);

  // Move down to the target pose
  target_tip_link_pose.pose.position.z -= .1;
  ROS_INFO_STREAM("Moving down to place object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = 0;
  // bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);
  // if (!success) 
  // {
    // ROS_INFO_STREAM("Linear motion plan to target place pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.01);  // Force the move even if LIN fails
  // }
  openGripper(robot_name, gripper_name);
  
  // // Move back up a little
  // target_tip_link_pose.pose.position.z += .05;
  // ROS_INFO_STREAM("Moving back up after placing object.");
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name);
  // if (!success) 
  // {
  //   ROS_INFO_STREAM("Linear motion plan back from place pose failed. Performing PTP.");
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  //   moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
  // }

  ROS_DEBUG_STREAM("Finished placing object.");
  return true;
}

bool SkillServer::pickFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name)
{
  publishMarker(target_tip_link_pose, "pick_pose");
  ROS_DEBUG_STREAM("Received pickFromAbove command.");
  
  // Move above the object
  openGripper(robot_name, gripper_name);
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Opening gripper, moving above object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, 1.0);

  // Move onto the object
  target_tip_link_pose.pose.position.z -= .1;
  ROS_INFO_STREAM("Moving down to pick object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);
  if (!success) 
  {
    ROS_INFO_STREAM("Linear motion plan to target pick pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);  // Force the move even if LIN fails
  }
  closeGripper(robot_name, gripper_name);

  // Move back up a little
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Moving back up after picking object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name);
  if (!success) 
  {
    ROS_INFO_STREAM("Linear motion plan back from pick pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
  }

  ROS_INFO_STREAM("Finished picking object.");
  return true;
}

// // For the screw insertion. Returns to the original position
// bool horizontal_spiral_wiggle(std::string robot_name, max_radius, start_pos, radius_increment = .001, speed = 0.02)
// {
//   group = self.groups[robot_name]
//   rospy.loginfo("Performing horizontal spiral motion " + str(speed))
//   rospy.loginfo("Setting velocity scaling to " + str(speed))
//   group.set_max_velocity_scaling_factor(speed)

//   # Modified code from Robotiq spiral search
//   theta_incr = 30
//   radius_inc_set = radius_increment / (360 / theta_incr)
//   r=0.0003  #Start radius
//   theta=0
//   RealRadius=0
  
//   # ==== MISBEHAVING VERSION (see https://answers.ros.org/question/300978/movegroupcommander-get_current_pose-returns-incorrect-result-when-using-real-robot/ )
//   # start_pos_bugged = group.get_current_pose() 
//   # ==== WORKING VERSION:
//   gripper_pos = geometry_msgs.msg.PoseStamped()
//   gripper_pos.header.frame_id = "a_bot_gripper_tip_link"
//   gripper_pos.pose.orientation.w = 1.0
//   # start_pos = self.listener.transformPose("world", gripper_pos)

//   next_pos = start_pos
//   while RealRadius <= max_radius and not rospy.is_shutdown():
//       #By default, the Spiral_Search function will maintain contact between both mating parts at all times
//       theta=theta+theta_incr
//       x=cos(radians(theta))*r
//       y=sin(radians(theta))*r
//       next_pos.pose.position.x = start_pos.pose.position.x + x
//       next_pos.pose.position.y = start_pos.pose.position.y + y
//       r=r + radius_inc_set
//       RealRadius = sqrt(pow(x,2)+pow(y,2))
//       self.go_to_pose_goal(robot_name, next_pos)
//       rospy.sleep(0.1)
//   # -------------
//   return True
// }

bool SkillServer::pickScrew(geometry_msgs::PoseStamped screw_head_pose, std::string screw_tool_id, std::string robot_name, std::string screw_tool_link, std::string fastening_tool_name)
{
  // Strategy: 
  // - Move 1 cm above the screw head pose
  // - Go down real slow for 2 cm while turning the motor in the direction that would loosen the screw
  // - Do a little circle motion with radius 1-2 mm
  // - Move up again slowly
  // - If the suction reports success, return true
  // - If not, display warning and return false?
  
  tf::Transform t;
  tf::Quaternion q(screw_head_pose.pose.orientation.x, screw_head_pose.pose.orientation.y, screw_head_pose.pose.orientation.z, screw_head_pose.pose.orientation.w);
  t.setOrigin(tf::Vector3(screw_head_pose.pose.position.x, screw_head_pose.pose.position.y, screw_head_pose.pose.position.z));
  t.setRotation(q);
  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), screw_head_pose.header.frame_id, "screw_pick_frame"));
  ROS_INFO_STREAM("Received pickScrew command.");
  
  // ROS_INFO_STREAM("Moving far above screw.");
  // screw_head_pose.pose.position.x = -.05;
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // moveToCartPoseLIN(screw_head_pose, robot_name, true, screw_tool_link, 1.0);

  ROS_INFO_STREAM("Moving close to screw.");
  screw_head_pose.pose.position.x = -.01;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = moveToCartPoseLIN(screw_head_pose, robot_name, true, screw_tool_link, 0.3);
  if (!success)
  {
    ROS_INFO_STREAM("Linear motion plan to target pick pose failed. Returning false.");
    return false;
  }

  planning_scene_interface_.allowCollisions(screw_tool_id, "tray_2_screw_holder");
  ROS_WARN_STREAM("TODO: TURN ON SUCTION");

  auto adjusted_pose = screw_head_pose;
  auto search_start_pose = screw_head_pose;
  bool screw_picked = false;
  
  double max_radius = .0025;
  double theta_incr = M_PI/3;
  double radius_increment = .001;
  double radius_inc_set = radius_increment / (2*M_PI / theta_incr);
  double r=0.0003;
  double theta=0;
  double RealRadius=0;
  double y, z;
  
  // Try to pick the screw, but go around in a spiral while trying to pick it
  while (!screw_picked)
  {
    sendFasteningToolCommand(fastening_tool_name, "loosen", false, 2.0);

    ROS_INFO_STREAM("Moving into screw.");
    adjusted_pose.pose.position.x = .01;
    moveToCartPoseLIN(adjusted_pose, robot_name, true, screw_tool_link, 0.05);

    ROS_INFO_STREAM("Moving back a bit slowly.");
    adjusted_pose.pose.position.x = -.01;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPoseLIN(adjusted_pose, robot_name, true, screw_tool_link, 0.1);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // TODO: Wait for message from suction topic
    // if (screw_tool_id == "screw_tool_m4")
    //   screw_picked = ros::topic::waitForMessage("/m4_tool/screw_suctioned", ros::Duration(1.0));
    // else if (screw_tool_id == "screw_tool_m3")
    //   screw_picked = ros::topic::waitForMessage("/m3_tool/screw_suctioned", ros::Duration(1.0));
    // else if (screw_tool_id == "screw_tool_m6")
    //   screw_picked = ros::topic::waitForMessage("/m6_tool/screw_suctioned", ros::Duration(1.0));
    ROS_WARN("Setting screw_picked to true");
    screw_picked = true;  // TODO: Replace this with the block above checking for pick success
    if ((RealRadius > max_radius) || (!ros::ok()))
      break;

    // Adjust the position (spiral search)
    ROS_INFO("Adjusting the position of the pick attempt slightly and retrying");
    theta=theta+theta_incr;
    y=cos(theta)*r;
    z=sin(theta)*r;
    adjusted_pose = search_start_pose;
    adjusted_pose.pose.position.y += y;
    adjusted_pose.pose.position.z += z;
    r = r + radius_inc_set;
    RealRadius = sqrt(pow(y,2)+pow(z,2));
  }

  planning_scene_interface_.disallowCollisions(screw_tool_id, "tray_2_screw_holder");
  ROS_INFO_STREAM("Moving back up completely.");
  screw_head_pose.pose.position.x = -.05;
  success = moveToCartPoseLIN(screw_head_pose, robot_name, true, screw_tool_link, 0.5);
  
  // TODO: Check suction success

  ROS_INFO_STREAM("Finished picking up screw. We don't know if we got it, so we are returning true for now. TODO.");
  return true;
}

bool SkillServer::publishMarker(geometry_msgs::PoseStamped marker_pose, std::string marker_type)
{
    visualization_msgs::Marker marker;
    marker.header = marker_pose.header;
    marker.header.stamp = ros::Time::now();
    marker.pose = marker_pose.pose;

    marker.ns = "markers";
    marker.id = marker_id_count++;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;

    if (marker_type == "pose")
    {
      publishPoseMarker(marker_pose);

      // Add a flat sphere
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .01;
      marker.scale.y = .05;
      marker.scale.z = .05;
      marker.color.g = 1.0;
      marker.color.a = 0.8;
      pubMarker_.publish(marker);
      return true;
    }
    if (marker_type == "place_pose")
    {
      publishPoseMarker(marker_pose);

      // Add a flat sphere
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .01;
      marker.scale.y = .05;
      marker.scale.z = .05;
      marker.color.g = 1.0;
      marker.color.a = 0.8;
      pubMarker_.publish(marker);
      return true;
    }
    if (marker_type == "pick_pose")
    {
      publishPoseMarker(marker_pose);

      // Add a flat sphere
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .01;
      marker.scale.y = .05;
      marker.scale.z = .05;
      marker.color.r = 0.8;
      marker.color.g = 0.4;
      marker.color.a = 0.8;
    }
    if (marker_type == "aist_vision_result")
    {
      // Add a sphere
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .01;
      marker.scale.y = .01;
      marker.scale.z = .01;
      marker.color.r = 0.8;
      marker.color.g = 0.4;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
    }
    else if (marker_type == "")
    {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = .02;
      marker.scale.y = .1;
      marker.scale.z = .1;
      
      marker.color.g = 1.0;
      marker.color.a = 0.8;
    }
    else 
    {ROS_WARN("No useful marker message received.");}
    pubMarker_.publish(marker);
    if (marker_id_count > 50) marker_id_count = 0;
    return true;
}

// This is a helper function for publishMarker. Publishes a TF-like frame.
bool SkillServer::publishPoseMarker(geometry_msgs::PoseStamped marker_pose)
{
  visualization_msgs::Marker marker;
  marker.header = marker_pose.header;
  marker.header.stamp = ros::Time::now();
  marker.pose = marker_pose.pose;

  marker.ns = "markers";
  marker.id = marker_id_count++;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;

  // This draws a TF-like frame.
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = .1;
  marker.scale.y = .01;
  marker.scale.z = .01;
  marker.color.a = .8;

  visualization_msgs::Marker arrow_x, arrow_y, arrow_z;
  arrow_x = marker; arrow_y = marker; arrow_z = marker;
  arrow_x.id = marker_id_count++; arrow_y.id = marker_id_count++; arrow_z.id = marker_id_count++;
  arrow_x.color.r = 1.0; arrow_y.color.g = 1.0; arrow_z.color.b = 1.0;

  rotatePoseByRPY(0, 0, M_PI/2, arrow_y.pose);
  rotatePoseByRPY(0, -M_PI/2, 0, arrow_z.pose);

  pubMarker_.publish(arrow_x); pubMarker_.publish(arrow_y); pubMarker_.publish(arrow_z);
  return true;
}
// ----------- Service definitions
bool SkillServer::goToNamedPoseCallback(o2as_msgs::goToNamedPose::Request &req,
                                           o2as_msgs::goToNamedPose::Response &res)
{
  ROS_INFO("Received goToNamedPose callback.");
  res.success = goToNamedPose(req.named_pose, req.planning_group);
  return true;
}

bool SkillServer::publishMarkerCallback(o2as_msgs::publishMarker::Request &req,
                                           o2as_msgs::publishMarker::Response &res)
{
  ROS_INFO("Received publishMarker callback.");
  return publishMarker(req.marker_pose, req.marker_type);
}
bool SkillServer::toggleCollisionsCallback(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res)
{
  ROS_INFO("Received toggleCollisions callback.");
  return toggleCollisions(req.data);
}
// ----------- Action servers

// alignAction
void SkillServer::executeAlign(const o2as_msgs::alignGoalConstPtr& goal)
{
  ROS_INFO("alignAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("alignAction is set as succeeded");
  alignActionServer_.setSucceeded();
}

// pickAction
void SkillServer::executePick(const o2as_msgs::pickGoalConstPtr& goal)
{
  ROS_INFO("pickAction was called");
  geometry_msgs::PoseStamped target_pose = goal->item_pose;
  if (target_pose.header.frame_id == "")
  {
    ROS_ERROR("FIXME");
    target_pose.header.frame_id = "world";
  }
  target_pose = transform_pose_now(target_pose, "world", tflistener_);

  if ((robot_statuses_[goal->robot_name].carrying_tool == true) && (goal->tool_name != "screw_tool"))
  {
    ROS_ERROR("Robot is already carrying a tool. Nothing can be picked except screws.");
    pickActionServer_.setAborted();
    return;
  }

  // TODO: Change the field name to something more sensible
  if (!goal->use_complex_planning)
  {
    ROS_INFO_STREAM("Using simple pick planning. Gripper will face downward, with z_axis_rotation = " << goal->z_axis_rotation);
    tf::Quaternion q_down, q_axis_rotation;
    q_down.setRPY(0, M_PI/2, 0);  // Faces into the table
    q_axis_rotation.setRPY(0, 0, goal->z_axis_rotation);
    tf::quaternionTFToMsg(q_down*q_axis_rotation, target_pose.pose.orientation);
    ROS_INFO_STREAM("Orientation set to: " << target_pose.pose.orientation.x << ", "
                                            << target_pose.pose.orientation.y << ", "
                                            << target_pose.pose.orientation.z << ", "
                                            << target_pose.pose.orientation.w);
  }
  
  if ((goal->gripper_command == "complex_pick_from_inside") || (goal->gripper_command == "complex_pick_from_outside"))
  {
    std::string end_effector_link_name = goal->robot_name + "_gripper_tip_link";
    // Do the positioning with the inner gripper first, then close the outer gripper
    publishMarker(target_pose, "pick_pose");
    ROS_INFO_STREAM("Doing complex pick with custom gripper.");
    
    // Move above the object
    openGripper("a_bot", "outer_gripper");
    if (goal->gripper_command == "complex_pick_from_inside")
    {
      ROS_INFO_STREAM("Opening outer and closing inner gripper, moving above object.");
      closeGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else if (goal->gripper_command == "complex_pick_from_outside")
    {
      ROS_INFO_STREAM("Opening outer and inner gripper, moving above object.");
      openGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    target_pose.pose.position.z += .1;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_pose, goal->robot_name, true, end_effector_link_name);

    ROS_INFO_STREAM("Moving closer to object (5 cm offset).");
    target_pose.pose.position.z -= .05;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    bool success = moveToCartPoseLIN(target_pose, goal->robot_name, true, end_effector_link_name, 0.1);

    target_pose.pose.position.z -= .05;
    ROS_INFO_STREAM("Moving down to object (no offset).");
    moveToCartPoseLIN(target_pose, goal->robot_name, true, end_effector_link_name, 0.01);  // Go very slow
    
    if (goal->gripper_command == "complex_pick_from_inside")
    {
      ROS_INFO_STREAM("Opening inner gripper to position object.");
      openGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else if (goal->gripper_command == "complex_pick_from_outside")
    {
      ROS_INFO_STREAM("Closing inner gripper to position object.");
      closeGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    closeGripper("a_bot", "outer_gripper");

    ROS_INFO_STREAM("Moving back up after picking object.");
    target_pose.pose.position.z += .1;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    success = moveToCartPoseLIN(target_pose, goal->robot_name, true, end_effector_link_name);
    if (!success) 
    {
      ROS_INFO_STREAM("Linear motion plan back from pick pose failed. Performing PTP.");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      moveToCartPosePTP(target_pose, goal->robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
    }

        if (goal->gripper_command == "complex_pick_from_inside")
    {
      ROS_INFO_STREAM("Closing inner gripper again after outer gripper grasped object.");
      closeGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else if (goal->gripper_command == "complex_pick_from_outside")
    {
      ROS_INFO_STREAM("Opening inner gripper again after outer gripper grasped object.");
      openGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    ROS_INFO_STREAM("Finished picking object.");
  }
  else if (goal->tool_name == "screw_tool")
  {
    goToNamedPose("screw_pick_ready", goal->robot_name);
    std::string screw_tool_id = "screw_tool_m" + std::to_string(goal->screw_size);
    std::string screw_tool_link = goal->robot_name + "_screw_tool_m" + std::to_string(goal->screw_size) + "_tip_link";
    std::string fastening_tool_name = "m" + std::to_string(goal->screw_size) +  "_tool";
    pickScrew(goal->item_pose, screw_tool_id, goal->robot_name, screw_tool_link, fastening_tool_name);
    goToNamedPose("screw_pick_ready", goal->robot_name);
  }
  else if (goal->tool_name == "suction")
  {;} // TODO: Here is space for code from AIST.
  else // No special tool being used; use just one gripper instead
  {
    // Warning: Using only "abs" rounds to int. Amazing.
    if ((std::abs(target_pose.pose.position.x)) > 0 || (std::abs(target_pose.pose.position.y)) > 0 || (std::abs(target_pose.pose.position.z)) > 0)
    {
      std::string ee_link_name;
      if (goal->robot_name == "a_bot"){ee_link_name = goal->robot_name + "_gripper_tip_link"; }
      else {ee_link_name = goal->robot_name + "_robotiq_85_tip_link";}
      pickFromAbove(target_pose, ee_link_name, goal->robot_name);
    }
    else
    {
      ROS_ERROR("Item_pose is empty and no tool_name was set. Not doing anything");
    }

    // TODO. The plan for complex planning: 
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
void SkillServer::executePlace(const o2as_msgs::placeGoalConstPtr& goal)
{
  ROS_INFO("placeAction was called");
  // TODO: Calculate the target pose with the item height currently held

  std::string ee_link_name;
  if (goal->robot_name == "a_bot"){ee_link_name = goal->robot_name + "_gripper_tip_link"; }
  else {ee_link_name = goal->robot_name + "_robotiq_85_tip_link";}
  
  if (goal->tool_name == "suction")
  {
    ; // TODO: Set the ee_link_name correctly and pass a flag to placeFromAbove
  }

  placeFromAbove(goal->item_pose, ee_link_name, goal->robot_name);
  ROS_INFO("placeAction is set as succeeded");
  placeActionServer_.setSucceeded();
}

// regraspAction
void SkillServer::executeRegrasp(const o2as_msgs::regraspGoalConstPtr& goal)
{
  ROS_INFO_STREAM("regraspAction was called with grasp_distance = " << goal->grasp_distance);
  openGripper(goal->receiver_robot_name);
  double gripper_distance_before_grasp = goal->gripper_distance_before_grasp, 
         grasp_distance = goal->grasp_distance, 
         gripper_distance_after_grasp = goal->gripper_distance_after_grasp;
  
  // Set default values
  if (gripper_distance_before_grasp < 0.001)
  {
    gripper_distance_before_grasp = 0.1;
  }
  if (grasp_distance == 0.0)
  {
    grasp_distance = 0.02;
  }
  if (gripper_distance_after_grasp < 0.001)
  {
    gripper_distance_after_grasp = 0.1;
  }

  // Create the handover_pose for Giver and Receiver robot
  std::string holder_robot_name = goal->giver_robot_name, 
              picker_robot_name = goal->receiver_robot_name;

  // The giver holds the item steady, the receiver picks it up.
  // Priority: c_bot, b_bot, a_bot (a_bot only gives passively)
  // If c_bot is involved, it always picks up (receiver), because its configuration is advantageous.
  if (holder_robot_name == "c_bot")
  {
    holder_robot_name = picker_robot_name;
    picker_robot_name = "c_bot";
  }
  else if ((holder_robot_name == "b_bot") && (picker_robot_name == "a_bot"))
  {
    holder_robot_name = "a_bot";
    picker_robot_name = "b_bot";
  }

  tf::Transform t;
  tf::Quaternion q;
  if (picker_robot_name == "c_bot")
  {
    t.setOrigin(tf::Vector3(-.28, .11, .55));  // x y z of the handover position
    double c_tilt = 0.0;
    if (holder_robot_name == "b_bot")
    { 
      c_tilt = 10.0;  // degrees. Tilts during the handover to c (this makes the pose nicer for b_bot)
    } 
    q.setRPY(M_PI, -c_tilt/180.0*M_PI, 0);   // r p y of the handover position (for the receiver)    
  }
  else if ((picker_robot_name == "b_bot") || (picker_robot_name == "a_bot"))
  {
    t.setOrigin(tf::Vector3(0.1, 0.0, 0.65)); 
    if (picker_robot_name == "b_bot")
    {
      q.setRPY(M_PI/2, 0, -M_PI/2);
      ROS_INFO_STREAM("picker_robot: " << picker_robot_name);
    }
    
    else
    {
      ROS_ERROR("This function should not arrive here.");
      q.setRPY(0, 0, 0);   
    }
  }
  t.setRotation(q);
  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "handover_frame"));

  geometry_msgs::PoseStamped handover_pose_holder, handover_pose_picker;
  handover_pose_holder.header.frame_id = "handover_frame";
  handover_pose_picker.header.frame_id = "handover_frame";
  handover_pose_picker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
  if(holder_robot_name == "a_bot")
  {
    handover_pose_holder.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI); // Facing the receiver, rotated
  }
  else handover_pose_holder.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, 0, M_PI); 
   
  publishMarker(handover_pose_picker, "pick_pose");
  publishMarker(handover_pose_holder, "place_pose");
  ros::Duration(.1).sleep();
  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "handover_frame"));

  goToNamedPose("back", picker_robot_name);
  if (holder_robot_name == "b_bot")
    goToNamedPose("regrasp_ready", holder_robot_name);

  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "handover_frame"));
  // Move the Giver to the regrasp_pose
  ROS_INFO_STREAM("Moving giver robot (" << holder_robot_name << ") to handover pose.");
  moveToCartPoseLIN(handover_pose_holder, holder_robot_name);
  
  // Move the Receiver to an approach pose, then on to grasp
  ROS_INFO_STREAM("Moving receiver robot (" << picker_robot_name << ") to approach pose.");
  handover_pose_picker.pose.position.x = -gripper_distance_before_grasp;
  moveToCartPoseLIN(handover_pose_picker, picker_robot_name);

  ros::Duration(1).sleep();
  tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "handover_frame"));
  ROS_INFO_STREAM("Moving receiver robot (" << picker_robot_name << ") to grasp pose.");
  handover_pose_picker.pose.position.x = -grasp_distance;
  ROS_WARN_STREAM("Position x is: " << -grasp_distance);
  moveToCartPoseLIN(handover_pose_picker, picker_robot_name);
  
  // Close the Receiver's gripper
  
  if(holder_robot_name != "a_bot")
  {
    closeGripper(goal->receiver_robot_name);
    ros::Duration(1).sleep();
    openGripper(goal->giver_robot_name);

    // Move back.
    ROS_INFO_STREAM("Moving receiver robot (" << picker_robot_name << ") back to approach pose.");
    handover_pose_picker.pose.position.x = -gripper_distance_after_grasp;
    moveToCartPoseLIN(handover_pose_picker, picker_robot_name);
    ROS_INFO("regraspAction is set as succeeded");
    regraspActionServer_.setSucceeded();
  }
}

// insertAction
void SkillServer::executeInsert(const o2as_msgs::insertGoalConstPtr& goal)
{
  ROS_INFO("insertAction was called");
  std::string ee_link_name;
  if (goal->active_robot_name == "a_bot"){ee_link_name = goal->active_robot_name + "_gripper_tip_link"; }
  else {ee_link_name = goal->active_robot_name + "_robotiq_85_tip_link";}

  bool inHandInsertion = true;
  if (inHandInsertion)  // = b_bot and c_bot.
  {
    // This bit is copied from the handover action. Assumes that the parts are perfectly aligned etc.
    tf::Transform t;
    tf::Quaternion q;
    t.setOrigin(tf::Vector3(-.28, .11, .55));  // x y z of the handover position
    double c_tilt = 10.0;
    q.setRPY(0, -c_tilt/180.0*M_PI, 0);   // r p y of the handover position (for the receiver)    
    t.setRotation(q);
    tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "in_hand_insertion_frame"));

    geometry_msgs::PoseStamped insertion_pose_active, insertion_pose_passive;
    insertion_pose_active.header.frame_id = "in_hand_insertion_frame";
    insertion_pose_passive.header.frame_id = "in_hand_insertion_frame";
    insertion_pose_active.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, 0, M_PI); // Facing the receiver, rotated

    std::string active_robot_name = "b_bot";
    std::string passive_robot_name = "c_bot";

    publishMarker(insertion_pose_passive, "pick_pose");
    publishMarker(insertion_pose_active, "place_pose");
    ros::Duration(.1).sleep();
    tfbroadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "workspace_center", "in_hand_insertion_frame"));

    goToNamedPose("back", passive_robot_name);

    // Move the 
    ROS_INFO_STREAM("Moving active robot (" << active_robot_name << ") to insertion pose.");
    moveToCartPosePTP(insertion_pose_active, active_robot_name);
    
    // Move the Receiver to an approach pose, then send the insertion command
    ROS_INFO_STREAM("Moving passive robot (" << passive_robot_name << ") to approach pose.");
    insertion_pose_passive.pose.position.x = - goal->starting_offset - .05;
    moveToCartPosePTP(insertion_pose_passive, passive_robot_name);

    o2as_msgs::sendScriptToUR srv;
    srv.request.program_id = "insertion";
    srv.request.robot_name = goal->active_robot_name;
    srv.request.max_insertion_distance = goal->max_insertion_distance;
    srv.request.max_force = goal->max_force;
    srv.request.max_radius = goal->max_radius;
    srv.request.radius_increment = goal->radius_increment;
    srv.request.forward_speed = goal->speed;
    sendScriptToURClient_.call(srv);
    if (srv.response.success == true)
      ROS_INFO("Successfully called the URScript client to start insertion");
    else
      ROS_WARN("Could not call the URScript client to start insertion");
    
    ROS_INFO("Waiting for the robot to finish the operation.");
    ros::Duration(.5).sleep();
    try{
      waitForURProgram("/" + active_robot_name + "_controller");
    }
    catch(ros::Exception e){;}
    

    // Assume that the operation succeeded
    // ROS_WARN("Sleeping for 15 seconds because we get no feedback from the robot.");
    // ros::Duration(15).sleep();
    openGripper(active_robot_name);

    // Move back
    ROS_INFO_STREAM("Moving passive robot (" << passive_robot_name << ") back.");
    insertion_pose_passive.pose.position.x = - goal->starting_offset - 0.1;
    moveToCartPosePTP(insertion_pose_passive, passive_robot_name);
  }

  // TODO: Add insertAction for one robot (not both at once)
  ROS_INFO("insertAction is set as succeeded");
  insertActionServer_.setSucceeded();
  return;
}

// screwAction
void SkillServer::executeScrew(const o2as_msgs::screwGoalConstPtr& goal)
{
  ROS_INFO("screwAction was called");

  // Set target pose for the end effector
  geometry_msgs::PoseStamped target_tip_link_pose = goal->target_hole;
  std::string screw_tool_link = goal->robot_name + "_screw_tool_" + "m" + std::to_string(goal->screw_size) + "_tip_link";
  // std::string screw_tool_link = held_screw_tool_ + "_tip";
  ROS_INFO_STREAM("screw tool link:  " << screw_tool_link);

  target_tip_link_pose.pose.position.x -= goal->screw_height+.005;  // Add the screw height and tolerance

  if (goal->screw_height < 0.001) {target_tip_link_pose.pose.position.x -= .02;}   // In case screw_height was not set

  // Move above the screw hole
  ROS_INFO("Moving above the screw hole.");
  target_tip_link_pose.pose.position.x -= .05;
  moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, screw_tool_link);

  sendFasteningToolCommand("m" + std::to_string(goal->screw_size) + "_tool", "tighten", false, 12.0, 600);
  // Disable collision for screw tool 
  updatePlanningScene();
  collision_detection::AllowedCollisionMatrix acm_original(planning_scene_.allowed_collision_matrix);
  planning_scene_interface_.allowCollisions("screw_tool_m" + std::to_string(goal->screw_size));

  // Move 1 cm into to the screw hole
  ROS_INFO("Pushing into the screw hole and doing spiral motion.");
  target_tip_link_pose.pose.position.x += .05 + .01;
  bool success = moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, screw_tool_link, 0.02);

  // Do spiral motion
  if (use_real_robot_)
  {
    o2as_msgs::sendScriptToUR srv;
    srv.request.program_id = "spiral_motion";
    srv.request.robot_name = goal->robot_name;
    srv.request.max_radius = .002;
    srv.request.radius_increment = .0005;
    srv.request.spiral_axis = "Y";
    sendScriptToURClient_.call(srv);
    if (srv.response.success == true)
    {
      ROS_DEBUG("Successfully called the service client to do spiral motion.");
      // waitForURProgram("/" + goal->robot_name +"_controller");
    }
    else
      ROS_ERROR("Could not call the service client to do spiral motion.");
  }

  // ROS_ERROR("Waiting for 20 seconds for screw spiral motion because the motors are not checked");
  // ros::Duration(30.0).sleep();
  bool finished_before_timeout = fastening_tool_client.waitForResult(ros::Duration(10.0));
  auto result = fastening_tool_client.getResult();
  ROS_INFO_STREAM("Screw tool motor command " << (finished_before_timeout ? "returned" : "did not return before timeout") <<". Result: " << result->control_result);

  // Move back up a little
  target_tip_link_pose.pose.position.x -= .05;
  success = moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, screw_tool_link);

  // TODO: If suction is lost, then the screw got stuck. Should try again at the same spot.
  
  // Enable collision for screw tool again
  moveit_msgs::PlanningScene ps_reset_collisions = planning_scene_;
  acm_original.getMessage(ps_reset_collisions.allowed_collision_matrix);
  planning_scene_interface_.applyPlanningScene(ps_reset_collisions);
  
  // TODO: Return success or not

  ROS_INFO("screwAction is set as succeeded");
  screwActionServer_.setSucceeded();
}

void SkillServer::executeChangeTool(const o2as_msgs::changeToolGoalConstPtr& goal)
{
  ROS_INFO("Received changeToolAction goal.");
  std::string equip_or_unequip = "equip";
  if (!goal->equip_the_tool) { equip_or_unequip = "unequip"; }

  std::string screw_tool_id = "screw_tool_m" + std::to_string(goal->screw_size);
  bool success = equipUnequipScrewTool(goal->robot_name, screw_tool_id, equip_or_unequip);
  
  if (success) { changeToolActionServer_.setSucceeded(); }
  else changeToolActionServer_.setAborted();
}

// ----------- End of the class definitions

int main(int argc, char **argv)
{
  ros::init(argc, argv, "o2as_skills");
  ros::AsyncSpinner spinner(1); // Needed for MoveIt to work.
  spinner.start();

  // Create an object of class SkillServer that will take care of everything
  SkillServer ss;
  ROS_INFO("O2AS skill server started");
  while (ros::ok())
  {
    ros::Duration(.1).sleep();
    ros::spinOnce();
  }
  
  return 0;
}
