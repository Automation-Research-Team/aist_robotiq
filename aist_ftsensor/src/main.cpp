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
	aist_ftsensor::ftsensor::InputWrenchType iwrench =
			aist_ftsensor::ftsensor::InputWrenchType::SUBSCRIBE;
        std::string input_wrench_type;
	if (ros::param::get("~input_wrench_type", input_wrench_type))
	{
	    if (! strcasecmp(input_wrench_type.c_str(), "socket"))
		iwrench = aist_ftsensor::ftsensor::InputWrenchType::SOCKET;
#if 0	/* same to default */
	    if (! strcasecmp(input_wrench_type.c_str(), "subscribe"))
		iwrench = aist_ftsensor::ftsensor::InputWrenchType::SUBSCRIBE;
#endif
        }
	ROS_INFO_STREAM("input_wrench_type[" << input_wrench_type << "]["
			<< iwrench << "]");

	aist_ftsensor::ftsensor	node("~", iwrench);
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
