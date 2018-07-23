

#ifndef O2AS_SKILL_SERVER_H
#define O2AS_SKILL_SERVER_H

#include "ros/ros.h"
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"

#include <tf/transform_listener.h>    // Includes the TF conversions

#include <chrono>
#include <thread>

// Services
#include "o2as_skills/goToNamedPose.h"

// Actions
#include <actionlib/server/simple_action_server.h>
#include "o2as_skills/pickAction.h"
#include "o2as_skills/placeAction.h"
#include "o2as_skills/insertAction.h"
#include "o2as_skills/screwAction.h"

class SkillServer
{
public:
  //Constructor
  SkillServer();

  //Helpers
  bool waitUntilArrived();      // Returns true if KUKA has arrived at destination
  bool moveToJointAnglesPTP(const double& j1, const double& j2, 
    const double& j3, const double& j4, const double& j5, const double& j6);
  bool moveToCartPosePTP(const double& x, const double& y, const double& z, 
  const double& u, const double& v, const double& w); // uvw = roll,pitch,yaw
  bool moveToCartPosePTP(geometry_msgs::Pose pose);
  bool stop();                  // Stops the robot at the current position

  // Callback declarations
  bool goToNamedPoseCallback(o2as_skills::goToNamedPose::Request &req,
                        o2as_skills::goToNamedPose::Response &res);

  // Actions
  void executePick(const o2as_skills::pickGoalConstPtr& goal);
  void executePlace(const o2as_skills::placeGoalConstPtr& goal);
  void executeInsert(const o2as_skills::insertGoalConstPtr& goal);
  void executeScrew(const o2as_skills::screwGoalConstPtr& goal);
  

private:
  ros::NodeHandle n_;

  // Service declarations
  ros::ServiceServer goToNamedPoseService;
  
  // Action declarations
  actionlib::SimpleActionServer<o2as_skills::pickAction> pickActionServer_;
  actionlib::SimpleActionServer<o2as_skills::placeAction> placeActionServer_;
  actionlib::SimpleActionServer<o2as_skills::insertAction> insertActionServer_;
  actionlib::SimpleActionServer<o2as_skills::screwAction> screwActionServer_;  

};//End of class SkillServer

#endif //O2AS_SKILL_SERVER_H