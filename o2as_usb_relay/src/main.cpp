/*!
 *   \file	main.cpp
 */
#include <ros/ros.h>
#include "o2as_usb_relay/SetPower.h"
#include "TU/USB++.h"

namespace o2as_usb_relay
{
struct USBHub : public TU::USBHub
{
    using	super = TU::USBHub;
    
    USBHub()	:super()				{}

    bool
    setPower(SetPower::Request& req, SetPower::Response& res)
    {
	ROS_INFO("Turning port %d %s.", req.port, (req.on ? "on" : "off"));

	try
	{
	    super::setPower(req.port, req.on);
	}
	catch (const std::exception& err)
	{
	    res.success = false;
	    res.message = err.what();

	    return false;
	}
		    
	res.success = true;
	res.message = std::string("Port ") + char('0' + req.port)
		    + " is turned " + (req.on ? "on." : "off.");

	return true;
    }
};
}	// namespace o2as_usb_relay

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "o2as_usb_relay_server");

    auto		hub = boost::make_shared<o2as_usb_relay::USBHub>();
    ros::NodeHandle	node("~");
    auto		service = node.advertiseService(
					"set_power",
					&o2as_usb_relay::USBHub::setPower,
					hub);
    ROS_INFO("o2as_usb_relay_server is active.");

    ros::spin();

    return 0;
}
