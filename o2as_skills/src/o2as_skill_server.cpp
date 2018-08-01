#include "o2as_skill_server.h"

SkillServer::SkillServer() : 
                  alignActionServer_(n_, "o2as_skills/align", boost::bind(&SkillServer::executeAlign, this, _1),false),
                  pickActionServer_(n_, "o2as_skills/pick", boost::bind(&SkillServer::executePick, this, _1),false),
                  placeActionServer_(n_, "o2as_skills/place", boost::bind(&SkillServer::executePlace, this, _1),false),
                  insertActionServer_(n_, "o2as_skills/insert", boost::bind(&SkillServer::executeInsert, this, _1),false),
                  screwActionServer_(n_, "o2as_skills/screw", boost::bind(&SkillServer::executeScrew, this, _1),false)
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
