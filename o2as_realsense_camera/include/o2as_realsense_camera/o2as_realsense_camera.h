#pragma once

#include <string>
#include <librealsense2/rs.hpp>
#include <ros/ros.h>

#include "o2as_realsense_camera/Connect.h"
#include "o2as_realsense_camera/SaveFrameForCadMatching.h"

namespace o2as {

const std::string CONNECT_SERVICE = "connect";
const std::string SAVE_FRAME_FOR_CAD_MATCHING_SERVICE = "save_frame_for_cad_matching";

class RealSenseCamera
{
public:
    RealSenseCamera(std::string serial_number);
    ~RealSenseCamera();

    bool polling();
    bool Connect(
        o2as_realsense_camera::Connect::Request &req,
        o2as_realsense_camera::Connect::Response &res);
    bool SaveFrameForCadMatching(
        o2as_realsense_camera::SaveFrameForCadMatching::Request &req,
        o2as_realsense_camera::SaveFrameForCadMatching::Response &res);

private:
    std::string serial_number_;
    rs2::pipeline pipe_;
    int color_width_ = 640;
    int color_height_ = 360;
    int depth_width_ = 640;
    int depth_height_ = 480;
    double depth_scale_ = 1.0;
    double inv_param_[9];

    ros::NodeHandle nh_;
    ros::ServiceServer service_connect_;
    ros::ServiceServer service_get_frame_;
    ros::ServiceServer service_save_frame_for_cad_matching_;
};

} // namespace o2as
