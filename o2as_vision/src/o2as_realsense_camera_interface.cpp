#include "o2as_realsense_camera_interface.h"
#include "o2as_realsense_camera/Connect.h"
#include "o2as_realsense_camera/SaveFrameForCadMatching.h"
//#include <o2as_realsense_camera/o2as_realsense_camera.h>

namespace o2as {

RealSenseCameraInterface::RealSenseCameraInterface(std::string group_name)
{
    ROS_DEBUG("RealSenseCameraInterface::RealSenseCameraInterface() begin");

    this->group_name_ = group_name;

	ros::service::waitForService(group_name+"/"+CONNECT_SERVICE, ros::Duration(-1));
   	this->connect_ = this->nh_.serviceClient<o2as_realsense_camera::Connect>(group_name+"/"+CONNECT_SERVICE);
	ros::service::waitForService(group_name+"/"+SAVE_FRAME_FOR_CAD_MATCHING_SERVICE, ros::Duration(-1));
   	this->save_frame_for_cad_matching_ = this->nh_.serviceClient<o2as_realsense_camera::SaveFrameForCadMatching>(group_name+"/"+SAVE_FRAME_FOR_CAD_MATCHING_SERVICE);

    ROS_DEBUG("RealSenseCameraInterface::RealSenseCameraInterface() end");
}

RealSenseCameraInterface::~RealSenseCameraInterface()
{
    ROS_DEBUG("RealSenseCameraInterface::~RealSenseCameraInterface()");
}

void RealSenseCameraInterface::SetImageDir(std::string image_dir)
{
    ROS_DEBUG("RealSenseCameraInterface::SetImageDir() begin");
    ROS_DEBUG("image_dir = %s", image_dir.c_str());

	this->image_dir_ = image_dir;
    this->pcloud_filename_ = this->image_dir_ + "/" + this->group_name_ + ".dat";
    this->image_filename_ = this->image_dir_ + "/" + this->group_name_ + ".png";

    ROS_DEBUG("RealSenseCameraInterface::SetImageDir() end");
}

bool RealSenseCameraInterface::Connect()
{
    ROS_DEBUG("RealSenseCameraInterface::Connect()");

	o2as_realsense_camera::Connect srv;
	if (this->connect_.call(srv)) {
		ROS_INFO("connect to camera success");
	} else {
		ROS_ERROR("connect to camera fail");
		return false;
	}

    ROS_DEBUG("RealSenseCameraInterface::Connect() end");
	return true;
}

bool RealSenseCameraInterface::SaveFrameForCadMatching()
{
    ROS_DEBUG("RealSenseCameraInterface::SaveFrameForCadMatching() begin");

	o2as_realsense_camera::SaveFrameForCadMatching srv;
	srv.request.pcloud_filename = this->pcloud_filename_;
	srv.request.image_filename  = this->image_filename_;
	if (this->save_frame_for_cad_matching_.call(srv)) {
		ROS_INFO("save frame success");
	} else {
		ROS_ERROR("save frame fail");
		return false;
	}

    ROS_DEBUG("RealSenseCameraInterface::SaveFrameForCadMatching() end");
	return true;
}

} // namespace o2as
