#pragma once

#include <ros/ros.h>

namespace o2as {

const std::string CONNECT_SERVICE = "connect";
const std::string SAVE_FRAME_FOR_CAD_MATCHING_SERVICE = "save_frame_for_cad_matching";

class RealSenseCameraInterface
{
public:
	RealSenseCameraInterface(std::string group_name);
	~RealSenseCameraInterface();

    bool Connect();
	bool SaveFrameForCadMatching();

	void SetImageDir(std::string image_dir);
	std::string GetPcloudFileName() { return this->pcloud_filename_; }
	std::string GetImageFileName() { return this->image_filename_; }

private:
    std::string group_name_;
    std::string image_dir_;
    std::string pcloud_filename_;
    std::string image_filename_;
	ros::NodeHandle nh_;
    ros::ServiceClient connect_;
    ros::ServiceClient save_frame_for_cad_matching_;
};

} // namespace o2as
