#include "o2as_skill_server.h"

SkillServer::SkillServer() : 
                  alignActionServer_(n_, "o2as_skills/align", boost::bind(&SkillServer::executeAlign, this, _1),false),
                  pickActionServer_(n_, "o2as_skills/pick", boost::bind(&SkillServer::executePick, this, _1),false),
                  placeActionServer_(n_, "o2as_skills/place", boost::bind(&SkillServer::executePlace, this, _1),false),
                  insertActionServer_(n_, "o2as_skills/insert", boost::bind(&SkillServer::executeInsert, this, _1),false),
                  screwActionServer_(n_, "o2as_skills/screw", boost::bind(&SkillServer::executeScrew, this, _1),false),
                  a_bot_group("a_bot"), b_bot_group("b_bot"), c_bot_group("c_bot"),
                  front_bots_group("front_bots"), all_bots_group("all_bots"),
                  b_bot_gripper_client("/b_bot_gripper/gripper_action_controller", true),
                  c_bot_gripper_client("/c_bot_gripper/gripper_action_controller", true)
                  // a_bot_gripper_client("/a_bot_gripper/gripper_action_controller", true),
{ 
  // Topics to publish

  // Services to advertise
  goToNamedPoseService = n_.advertiseService("o2as_skills/goToNamedPose", &SkillServer::goToNamedPoseCallback,
                                        this);

  // Actions we serve
  alignActionServer_.start();
  pickActionServer_.start();
  placeActionServer_.start();
  insertActionServer_.start();
  screwActionServer_.start();

  // Action clients
  ROS_INFO("Waiting for action servers to start.");
  // a_bot_gripper_client.waitForServer(); 
  b_bot_gripper_client.waitForServer();
  c_bot_gripper_client.waitForServer();
  ROS_INFO("Action servers started.");

  // Set up MoveGroups
  a_bot_group.setPlanningTime(0.5);
  a_bot_group.setPlannerId("RRTConnectkConfigDefault");
  a_bot_group.setEndEffectorLink("a_bot_robotiq_85_tip_link");
  b_bot_group.setPlanningTime(0.5);
  b_bot_group.setPlannerId("RRTConnectkConfigDefault");
  b_bot_group.setEndEffectorLink("b_bot_robotiq_85_tip_link");
  c_bot_group.setPlanningTime(0.5);
  c_bot_group.setPlannerId("RRTConnectkConfigDefault");
  c_bot_group.setEndEffectorLink("c_bot_robotiq_85_tip_link");
  front_bots_group.setPlanningTime(2.0);
  front_bots_group.setPlannerId("RRTConnectkConfigDefault");
  all_bots_group.setPlanningTime(3.0);
  all_bots_group.setPlannerId("RRTConnectkConfigDefault");
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
  
  success_plan = group_pointer->plan(myplan);
  if (success_plan) 
  {
    if (wait) motion_done = group_pointer->execute(myplan);
    else motion_done = group_pointer->asyncExecute(myplan);
    if (motion_done) {return true;}
  }
  return false;
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
  if (robot_name == "a_bot") return &a_bot_group;
  if (robot_name == "b_bot") return &b_bot_group;
  if (robot_name == "c_bot") return &c_bot_group;
  if (robot_name == "front_bots") return &front_bots_group;
  if (robot_name == "all_bots") return &all_bots_group;
}


// ----------- Internal functions

bool SkillServer::equipScrewTool(std::string robot_name, std::string screw_tool_id)
{
  if (holding_object) 
  {
    ROS_INFO("Robot already holds an object. Cannot pick up screw tool.");
    return false;
  }

  spawnTool("screw_tool_m5");
  
  // Plan & execute motion to in front of holder
  geometry_msgs::PoseStamped ps_approach, ps_pickup;
  ps_approach.header.frame_id = "screw_tool_m5_link";
  ps_approach.pose.position.y = .1;
  ps_approach.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI / 2);
  moveToCartPosePTP(ps_approach, "b_bot");

  // Open gripper the correct amount
  openGripper("b_bot");
  
  // Plan & execute LINEAR motion to the tool change position
  // Close gripper, attach the tool object to the gripper in the Planning Scene
  // MAYBE: Set the ACM in the planning scene up
  // Plan & execute LINEAR motion away from the tool change position
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
      b_bot_gripper_client.sendGoal(goal);
      finished_before_timeout = b_bot_gripper_client.waitForResult(ros::Duration(5.0));
    }
    else if (robot_name == "c_bot")
    {
      c_bot_gripper_client.sendGoal(goal);
      finished_before_timeout = c_bot_gripper_client.waitForResult(ros::Duration(5.0));
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

    // Define the holder as a collision object
    collision_objects[0].header.frame_id = "c_bot_base_smfl";
    collision_objects[0].id = screw_tool_id;

    collision_objects[0].primitives.resize(3);
    collision_objects[0].primitive_poses.resize(3);
    // The bit cushion and motor
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(2);
    collision_objects[0].primitives[0].dimensions[0] = 0.026;
    collision_objects[0].primitives[0].dimensions[1] = 0.04;
    collision_objects[0].primitives[0].dimensions[2] = 0.055;
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = -0.01;
    collision_objects[0].primitive_poses[0].position.z = 0.0275;

    // The "shaft" + suction attachment
    collision_objects[0].primitives[1].type = collision_objects[0].primitives[1].BOX;
    collision_objects[0].primitives[1].dimensions.resize(2);
    collision_objects[0].primitives[1].dimensions[0] = 0.02;
    collision_objects[0].primitives[1].dimensions[1] = 0.028;
    collision_objects[0].primitives[1].dimensions[2] = 0.08;
    collision_objects[0].primitive_poses[1].position.x = 0;
    collision_objects[0].primitive_poses[1].position.y = -0.0115;  // 21 mm distance from axis
    collision_objects[0].primitive_poses[1].position.z = -0.04;

    // The cylinder representing the tip
    collision_objects[0].primitives[2].type = collision_objects[0].primitives[2].CYLINDER;
    collision_objects[0].primitives[2].dimensions.resize(2);
    collision_objects[0].primitives[2].dimensions[0] = 0.02;    // Cylinder height
    collision_objects[0].primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
    collision_objects[0].primitive_poses[2].position.x = 0;
    collision_objects[0].primitive_poses[2].position.y = 0;  // 21 mm distance from axis
    collision_objects[0].primitive_poses[2].position.z = -0.09;

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);;
}

// Remove the tool from the scene so it does not cause unnecessary collision calculations
bool SkillServer::despawnTool(std::string screw_tool_id)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = screw_tool_id;
    collision_objects[0].operation = collision_objects[0].REMOVE;
    planning_scene_interface.applyCollisionObjects(collision_objects);;
}


// ----------- Service definitions
bool SkillServer::goToNamedPoseCallback(o2as_skills::goToNamedPose::Request &req,
                                           o2as_skills::goToNamedPose::Response &res)
{
  ROS_INFO("goToNamedPoseCallback was called");
  // TODO: Insert the commands
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
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

  // Create an object of class SkillServer that will take care of everything
  SkillServer o2as_skill_server;

  ROS_INFO("O2AS skill server started");

  ros::spin();
  return 0;
}
