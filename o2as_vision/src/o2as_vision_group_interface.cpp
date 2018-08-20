#include "o2as_vision_group_interface.h"

namespace o2as {

VisionGroupInterface::VisionGroupInterface(std::string name) :
	realsense_camera_(name), cad_matching_(name), planning_scene_(name)
{
    this->name_ = name;
}

VisionGroupInterface::~VisionGroupInterface()
{
}

void VisionGroupInterface::SetImageDir(std::string image_dir)
{
	ROS_DEBUG("VisionGroupInterface::SetImageDir()");

	this->realsense_camera_.SetImageDir(image_dir);
    this->pcloud_filename_ = this->realsense_camera_.GetPcloudFileName();
    this->image_filename_ = this->realsense_camera_.GetImageFileName();

	ROS_DEBUG("VisionGroupInterface::SetImageDir() end");	
}

bool VisionGroupInterface::Prepare()
{
	ROS_DEBUG("VisionGroupInterface::Prepare()");

	ROS_INFO("load model data for cad matching");
	if (!this->cad_matching_.LoadModelData()) {
		ROS_ERROR("cad matching load model fail");
	}
	ROS_INFO("prepare cad matching");
	if (!this->cad_matching_.Prepare()) {
		ROS_ERROR("cad matching prepare fail");
	}
	ROS_INFO("connect to camera");
	if (!this->realsense_camera_.Connect()) {
		ROS_ERROR("camera connect fail");
	}

	ROS_DEBUG("VisionGroupInterface::prepare() end");	
}

void VisionGroupInterface::AddDetectedObjectToPlanningScene(const o2as_cad_matching_msgs::DetectedObject& object)
{
	ROS_DEBUG("VisionGroupInterface::AddDetectedObjectToPlanningScene() begin");

	// convert position and orientation of detected object
	ROS_DEBUG("pos = (%f, %f, %f)", object.pos3D.x, object.pos3D.y, object.pos3D.z);
	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position = object.pos3D;
	pose.orientation.w = 1.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;

	// convert size 
	float mm2meter = 0.001; // [mm to meter]
	float scale[3] = { mm2meter, mm2meter, mm2meter };

	// add mesh to planning scene
	std::string name = "05_MBRFA30-2-P6";
	std::string cad_filename = "package://o2as_parts_description/meshes/05_MBRFA30-2-P6.dae";
	this->planning_scene_.AddMesh(name+"_mesh", pose, cad_filename, scale);

	// // add box to planning scene (test)
	// float len = 0.05;
	// float box_size[3] = { len, len, len };
	// this->planning_scene_.AddBox(name+"_box", pose, box_size);

	ROS_DEBUG("VisionGroupInterface::AddDetectedObjectToPlanningScene() end");
}

bool VisionGroupInterface::FindObject()
{
	ROS_DEBUG("VisionGroupInterface::FindObject() begin");
	
    // get image from sensor
	ROS_INFO("save frame for cad matching");	
	this->realsense_camera_.SaveFrameForCadMatching();
	
    // cad matching
	ROS_INFO("search object");	
	o2as_cad_matching_msgs::SearchResult result;
	this->cad_matching_.Search(this->pcloud_filename_, this->image_filename_, result);
	
    // add detected objects to the planning scene
	if (result.result_num == 0)	{
		ROS_INFO("no object found");
	} else {
		ROS_INFO("add detected object to planning scene");	
		AddDetectedObjectToPlanningScene(result.detected_objects[0]);
	}

	ROS_DEBUG("VisionGroupInterface::FindObject() end");
}

} // namespace o2as
