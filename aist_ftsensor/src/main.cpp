/*!
 *  \file	main.cpp
 *  \brief	wrench publisher for force sensors
 */
#include "ftsensor.h"
#include <ros/param.h>
#include <string.h>

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "aist_ftsensor");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	aist_ftsensor::ftsensor::Input input =
				    aist_ftsensor::ftsensor::Input::TOPIC;
        std::string param_input;
	if (ros::param::get("~input", param_input))
	{
	    if (! strcasecmp(param_input.c_str(), "socket"))
		input = aist_ftsensor::ftsensor::Input::SOCKET;
#if 0	/* same to default */
	    if (! strcasecmp(param_input.c_str(), "topic"))
		input = aist_ftsensor::ftsensor::Input::TOPIC;
#endif
        }
	ROS_INFO_STREAM("input[" << param_input << "][" << input << "]");

	aist_ftsensor::ftsensor	node("~", input);
	node.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }
    catch (...)
    {
	std::cerr << "Unknown error." << std::endl;
	return 1;
    }

    return 0;
}
