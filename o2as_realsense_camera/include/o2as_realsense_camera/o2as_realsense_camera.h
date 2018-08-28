#pragma once

#include <string>
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <o2as_realsense_camera/CameraSettingConfig.h>

#include "o2as_realsense_camera/Connect.h"
#include "o2as_realsense_camera/SaveCameraInfo.h"
#include "o2as_realsense_camera/SaveFrameForCadMatching.h"

namespace o2as {

const std::string CONNECT_SERVICE = "connect";
const std::string SAVE_FRAME_FOR_CAD_MATCHING_SERVICE = "save_frame_for_cad_matching";
const std::string SAVE_CAMERA_INFO_SERVICE = "save_camera_info";

const std::string COLOR_IMAGE_TOPIC = "color/image_raw";
const std::string DEPTH_IMAGE_TOPIC = "depth/image_raw";

class RealSenseCamera
{
public:
    RealSenseCamera(std::string serial_number, int color_width, int color_height, int depth_width, int depth_height);
    ~RealSenseCamera();
    void dynamic_reconfigure_callback(o2as_realsense_camera::CameraSettingConfig &config, uint32_t level);

    bool executeConnect(
        o2as_realsense_camera::Connect::Request &req,
        o2as_realsense_camera::Connect::Response &res);
    bool executeSaveFrameForCadMatching(
        o2as_realsense_camera::SaveFrameForCadMatching::Request &req,
        o2as_realsense_camera::SaveFrameForCadMatching::Response &res);
    bool executeSaveCameraInfo(
        o2as_realsense_camera::SaveCameraInfo::Request &req,
        o2as_realsense_camera::SaveCameraInfo::Response &res);
    bool polling();

private:
    void enableDisplay();
    void disableDisplay();

    bool connect();
    bool disconnect();
    bool prepareDataConversion();
    bool getAlignedFrameSet(rs2::frame& depth_frame, rs2::frame& color_frame);
    bool saveDepthFrameInCadFormat(std::string pcloud_filename, rs2::frame& depth_frame);
    bool saveColorFrameInCadFormat(std::string image_filename, const rs2::frame& color_frame);
    bool saveFrameForCadMatching(std::string pcloud_filename, std::string image_filename);
    rs2_intrinsics getColorCameraIntrinsics();
    bool saveCameraInfo(std::string filename);

protected:
    bool display_;
    bool publish_;

    std::string serial_number_;
    rs2::pipeline pipe_;
    int color_width_;
    int color_height_;
    int depth_width_;
    int depth_height_;
    bool connected_;
    double depth_scale_;
    double inv_param_[9];

    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<o2as_realsense_camera::CameraSettingConfig> dynamic_reconfigure_server;
    ros::ServiceServer connect_service_;
    ros::ServiceServer save_frame_for_cad_matching_service_;
    ros::ServiceServer save_camera_info_service_;
    ros::Publisher color_image_pub_;
	ros::Publisher depth_image_pub_;
};

} // namespace o2as
