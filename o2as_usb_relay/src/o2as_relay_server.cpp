/*!
 *   \file	o2as_relay_server.cpp
 */
#include "ros/ros.h"
#include "o2as_usb_relay/SetPower.h"
#include "TU/USB++.h"

namespace TU
{
struct USBHubAdaptor : public USBHub
{
    USBHubAdaptor()	:USBHub()				{}

    bool
    setPower(o2as_usb_relay::SetPower::Request&  req,
	     o2as_usb_relay::SetPower::Response& res)
    {
	ROS_INFO("Turning port %d %s.", req.port, (req.on ? "on" : "off"));

	try
	{
	    USBHub::setPower(req.port, req.on);
	}
	catch (const std::exception& err)
	{
	    res.success = false;
	    res.message = err.what();
	}
		    
	res.success = true;
	res.message = std::string("Port ") + char('0' + req.port)
		    + " is turned " + (req.on ? "on." : "off.");

	return true;
    }
};
}	// namespace TU

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "o2as_usb_relay_server");

    auto		hub = boost::make_shared<TU::USBHubAdaptor>();
    ros::NodeHandle	node;
    auto		service = node.advertiseService(
					"turn_on_gripper_suction",
					&TU::USBHubAdaptor::setPower, hub);
    ROS_INFO("o2as_usb_relay_server is active.");

    ros::spin();

    return 0;
}
