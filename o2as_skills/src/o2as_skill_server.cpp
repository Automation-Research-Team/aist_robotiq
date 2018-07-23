#include "o2as_skill_server.h"

SkillServer::SkillServer() : pickActionServer_(n_, "o2as_skills/pick", boost::bind(&SkillServer::execute, this, _1),false)
{ 
  // Topics to publish

  // Services to advertise
  goToNamedPoseService = n_.advertiseService("o2as_skills/goToNamedPose", &SkillServer::goToNamedPoseCallback,
                                        this);

  // Actions we serve
  pickActionServer_.start();
  // placeActionServer_.start();
  // insertActionServer_.start();
  // screwActionServer_.start();
}

// TODO: Write this function/decide if it is needed
bool SkillServer::waitUntilArrived()
{
    ROS_INFO("Waiting for robot to reach its destination");
    return true;
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

// pickAction
void SkillServer::execute(const o2as_skills::pickGoalConstPtr& goal)
{
  ROS_INFO("pickAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("pickAction is set as succeeded");
  pickActionServer_.setSucceeded();
}

// // placeAction
// void SkillServer::execute(const o2as_skills::placeGoalConstPtr& goal)
// {
//   ROS_INFO("placeAction was called");
//   // TODO: Insert commands to do the action
//   std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//   ROS_INFO("placeAction is set as succeeded");
//   placeActionServer_.setSucceeded();
// }

// // insertAction
// void SkillServer::execute(const o2as_skills::insertGoalConstPtr& goal)
// {
//   ROS_INFO("insertAction was called");
//   // TODO: Insert commands to do the action
//   std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//   ROS_INFO("insertAction is set as succeeded");
//   insertActionServer_.setSucceeded();
// }

// // screwAction
// void SkillServer::execute(const o2as_skills::screwGoalConstPtr& goal)
// {
//   ROS_INFO("screwAction was called");
//   // TODO: Insert commands to do the action
//   std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//   ROS_INFO("screwAction is set as succeeded");
//   screwActionServer_.setSucceeded();
// }


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
