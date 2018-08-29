#include "o2as_realsense_camera/o2as_realsense_camera.h"

#include <string>
#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "o2as_realsense_camera/util_cam.h"

using namespace o2as;

//#define LOG_LEVEL ros::console::levels::Debug
#define LOG_LEVEL ros::console::levels::Info

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "realsense_camera_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, LOG_LEVEL);
    ros::NodeHandle nh;

    // check existence of device with specified serial no.
    std::string serial_number;
    ros::param::get("~serial_number", serial_number);
    if (!DeviceExists(serial_number)) {
        ROS_ERROR("realsense device with serial number %s not exists", serial_number.c_str());
        return -1;
    }

    // get image size setting
    int color_width, color_height, depth_width, depth_height;
    ros::param::get("~color_width",  color_width);
    ros::param::get("~color_height", color_height);
    ros::param::get("~depth_width",  depth_width);
    ros::param::get("~depth_height", depth_height);

    try {
        // create inastance of camera server
        RealSenseCamera device(serial_number, color_width, color_height, depth_width, depth_height);
		while (ros::ok()) {
            //server.polling();
            cv::waitKey(1);
            ros::spinOnce();
        }
    }
	catch(const char *error_message) {
		ROS_ERROR("%s", error_message);
	}
}
