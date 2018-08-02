#include "o2as_skill_server.h"

SkillServer::SkillServer() : 
                  alignActionServer_(n_, "o2as_skills/align", boost::bind(&SkillServer::executeAlign, this, _1),false),
                  pickActionServer_(n_, "o2as_skills/pick", boost::bind(&SkillServer::executePick, this, _1),false),
                  placeActionServer_(n_, "o2as_skills/place", boost::bind(&SkillServer::executePlace, this, _1),false),
                  insertActionServer_(n_, "o2as_skills/insert", boost::bind(&SkillServer::executeInsert, this, _1),false),
                  screwActionServer_(n_, "o2as_skills/screw", boost::bind(&SkillServer::executeScrew, this, _1),false),
                  a_bot_group("a_bot"), b_bot_group("b_bot"), c_bot_group("c_bot"),
                  front_bots_group("front_bots"), all_bots_group("all_bots")
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
}


// TODO: Write this function/decide if it is needed
bool SkillServer::moveToJointAnglesPTP(const double& j1, const double& j2, 
    const double& j3, const double& j4, const double& j5, const double& j6)
{
  return true;
}


// Publishes a cartesian pose for the robot to move to.
bool SkillServer::moveToCartPosePTP(const double& x, const double& y, const double& z, 
  const double& u, const double& v, const double& w)
{
  // Create the pose from the input parameters
  // Prepare components
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;

  geometry_msgs::Quaternion q;
  tf::Quaternion qt;
  qt.setRPY(u, v, w);
  tf::quaternionTFToMsg(qt, q);

  // Create the pose
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;
  
  return moveToCartPosePTP(pose);
}

// TODO: Write this function/decide if it is needed
bool SkillServer::moveToCartPosePTP(geometry_msgs::Pose pose)
{
  return true;
}

// TODO: Write this function/decide if it is needed
bool SkillServer::stop()
{
  return true;
}


// ----------- Internal functions

bool SkillServer::equipScrewTool(int screw_tool_size)
{
  ;
  // --- THE PLAN:
  // Spawn the tool 
  // Make sure no item is held
  // Plan & execute motion to in front of holder
  // Open gripper the correct amount
  // Plan & execute LINEAR motion to the tool change position
  // Close gripper, attach the tool object to the gripper in the Planning Scene
  // MAYBE: Set the ACM in the planning scene up
  // Plan & execute LINEAR motion away from the tool change position
  // Optional: Move back to home
}

bool SkillServer::putBackScrewTool()
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

// Add the screw tool as a Collision Object to the scene, so that it can be attached to the robot
bool SkillServer::spawnTool(int screw_tool_size)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Define the object
    collision_objects[0].header.frame_id = "c_bot_base_smfl";
    collision_objects[0].id = "screw_tool_m5"; //TODO: Switch name

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type =
        collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.04;
    collision_objects[0].primitives[0].dimensions[1] = 0.025;
    collision_objects[0].primitives[0].dimensions[2] = 0.155;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = .6;
    collision_objects[0].primitive_poses[0].position.y = .8;
    collision_objects[0].primitive_poses[0].position.z = 0.06;
    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);;
}

// Remove the tool from the scene so it does not cause unnecessary collision calculations
bool SkillServer::despawnTool(int screw_tool_size)
{
  ;
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
