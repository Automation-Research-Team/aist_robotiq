#include "o2as_realsense_camera/o2as_realsense_camera.h"

#include <string>
#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "o2as_realsense_camera/util_cam.h"

using namespace o2as;

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "realsense_camera_node");
    ros::NodeHandle nh;

    std::string serial_number;
    ros::param::get("~serial_number", serial_number);
    if (!DeviceExists(serial_number)) {
        ROS_ERROR("realsense device with serial number %s not exists", serial_number.c_str());
        return -1;
    }

    try {
        RealSenseCamera device(serial_number);
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
