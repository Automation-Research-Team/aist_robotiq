#include "o2as_planning_scene_interface.h"

//#include <Eigen/Eigen>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

namespace o2as {

PlanningSceneInterface::PlanningSceneInterface(std::string group_name)
{
	ROS_DEBUG("PlanningSceneInterface::PlanningSceneInterface() begin");

    this->group_name_ = group_name;
	this->camera_frame_ = "/" + this->group_name_ + "_depth_frame";

	// scene publisher
	ros::Duration sleep_time(1.0);
	this->planning_scene_diff_publisher_ = this->nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

	ROS_DEBUG("PlanningSceneInterface::PlanningSceneInterface() end");
}

PlanningSceneInterface::~PlanningSceneInterface()
{
	ROS_DEBUG("PlanningSceneInterface::~PlanningSceneInterface");
}

inline Eigen::Vector3d toEigenVec(float size[3])
{
	return Eigen::Vector3d(size[0], size[1], size[2]);
}

moveit_msgs::CollisionObject PlanningSceneInterface::MakeBox(std::string name, geometry_msgs::Pose pose, float size[3])
{
	ROS_DEBUG("PlanningSceneInterface::MakeBox() begin");

	// create collision object message with mesh
	moveit_msgs::CollisionObject co;
	co.header.frame_id = this->camera_frame_;
	co.operation = co.ADD;
	Eigen::Vector3d scale(0.001, 0.001, 0.001);
	shape_msgs::SolidPrimitive shape;
	shape.type = shape.BOX;
	shape.dimensions.resize(3);
	shape.dimensions[0] = size[0];
	shape.dimensions[1] = size[1];
	shape.dimensions[2] = size[2];
	co.primitives.push_back(shape);
	co.primitive_poses.push_back(pose);

	ROS_DEBUG("PlanningSceneInterface::MakeBox() end");
	return co;
}

void PlanningSceneInterface::AddBox(std::string name, geometry_msgs::Pose pose, float size[3])
{
	ROS_DEBUG("PlanningSceneInterface::AddBox() begin");

	// publish scene change message
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(MakeBox(name, pose, size));
	planning_scene.is_diff = true;
	this->planning_scene_diff_publisher_.publish(planning_scene);

	ROS_DEBUG("PlanningSceneInterface::AddBox() end");
}

moveit_msgs::CollisionObject PlanningSceneInterface::MakeMesh(std::string name, geometry_msgs::Pose pose, std::string cad_filename, float size[3])
{
	ROS_DEBUG("PlanningSceneInterface::MakeMesh() begin");

	// create collision object message with mesh
	moveit_msgs::CollisionObject co;
	co.header.frame_id = this->camera_frame_;
	co.operation = co.ADD;
	const shapes::Mesh* shape = shapes::createMeshFromResource(cad_filename, toEigenVec(size));
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(shape, shape_msg);
	shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
	co.meshes.push_back(mesh_msg);
	co.mesh_poses.resize(1);
	co.mesh_poses[0] = pose;

	ROS_DEBUG("PlanningSceneInterface::MakeMesh() end");
	return co;
}

void PlanningSceneInterface::AddMesh(std::string name, geometry_msgs::Pose pose, std::string cad_filename, float size[3])
{
	ROS_DEBUG("PlanningSceneInterface::AddMesh() begin");

	// publish scene change message
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(MakeMesh(name, pose, cad_filename, size));
	planning_scene.is_diff = true;
	this->planning_scene_diff_publisher_.publish(planning_scene);

	ROS_DEBUG("PlanningSceneInterface::AddMesh() end");
}

} // namespace o2as
