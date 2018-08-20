#pragma once

#include <string>
#include <ros/ros.h>
#include "o2as_realsense_camera_interface.h"
#include "o2as_cad_matching_interface.h"
#include "o2as_planning_scene_interface.h"
#include "o2as_cad_matching_msgs/DetectedObject.h"

namespace o2as {

class VisionGroupInterface
{
public:
	VisionGroupInterface(std::string name_);
	~VisionGroupInterface();

	bool Prepare();
	bool FindObject();
	void AddDetectedObjectToPlanningScene(const o2as_cad_matching_msgs::DetectedObject& object);

    std::string name() { return name_; }
	void SetImageDir(std::string image_dir);

protected:
    std::string name_;
    std::string pcloud_filename_;
    std::string image_filename_;
    ros::NodeHandle nh_;
	RealSenseCameraInterface realsense_camera_;
	CadMatchingInterface cad_matching_;
	PlanningSceneInterface planning_scene_;
};

} // namespace o2as
