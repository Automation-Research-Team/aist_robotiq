#include "ros/ros.h"
#include "std_srvs/SetBool.h"

// This node is based on the tutorial: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

bool turn_on_relay_1(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res)
{
    if (req.data == true)
    {
        ROS_INFO("Turning suction on.");
        // Insert code to turn on suction here
    }
    else 
    {
        ROS_INFO("Turning suction off.");
        // Insert code to turn off suction here
    }

    res.success = true;
    res.message = "";
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_relay_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("turn_on_gripper_suction", turn_on_relay_1);
  ROS_INFO("USB relay server is active.");
  ros::spin();

  return 0;
}
