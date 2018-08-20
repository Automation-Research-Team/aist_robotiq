#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

namespace o2as {

class PlanningSceneInterface
{
public:
	PlanningSceneInterface(std::string group_name);
	~PlanningSceneInterface();

    moveit_msgs::CollisionObject MakeBox(std::string name, geometry_msgs::Pose pose, float size[3]);
    void AddBox(std::string name, geometry_msgs::Pose pose, float size[3]);
    moveit_msgs::CollisionObject MakeMesh(std::string name, geometry_msgs::Pose pose, std::string filename, float size[3]);
    void AddMesh(std::string name, geometry_msgs::Pose pose, std::string cad_filename, float size[3]);

protected:
    std::string group_name_;
    std::string camera_frame_;
    ros::NodeHandle nh_;
    ros::Publisher planning_scene_diff_publisher_;
};

} // namespace o2as
