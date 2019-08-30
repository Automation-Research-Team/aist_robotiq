/*!
 *  \file	my_debug.cpp
 *  \brief	for debug
 */
#include "ftsensor.h"
#include <ros/package.h>
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

	std::ifstream dump(node.DBG_DUMP_FILE);
	ROS_INFO_STREAM("load file=" << node.DBG_DUMP_FILE);
	std::string buf;
	while (dump && std::getline(dump, buf))
	{
	    ROS_INFO_STREAM("load[" << buf << "]");
	    std::string::size_type p[2] = { 0, 0 };
	    p[1] = buf.find(" ");
	    std::vector<double> vec;
	    while (p[1] != std::string::npos)
	    {
		vec.push_back(std::stod(buf.substr(p[0], p[1]-p[0])));
		p[0] = p[1]+1;
		p[1] = buf.find(" ", p[0]);
	    }
	    if (buf.size() - p[0] > 0)
		vec.push_back(std::stod(buf.substr(p[0])));
	    for (int i = 0; i < vec.size(); i++)
		ROS_INFO_STREAM("load [" << i << "][" << vec[i] << "]");
	    if (vec.size() < 9)
		continue;

	    geometry_msgs::Vector3 f, m;
	    Eigen::Matrix<double, 3, 1> k;
	    f.x = vec[0]; f.y = vec[1]; f.z = vec[2];
	    m.x = vec[3]; m.y = vec[4]; m.z = vec[5];
	    k << vec[6], vec[7], vec[8];

	    node.dbg_take_sample(k, f, m);
	}
	std_srvs::Trigger::Request  req;    // dummy
	std_srvs::Trigger::Response res;    // dummy
	node.compute_calibration_callback(req, res);
	node.save_calibration_callback(req, res);
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
