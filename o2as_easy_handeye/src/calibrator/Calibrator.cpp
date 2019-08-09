/*!
  \file	 Calibrator.cpp
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/
#include <fstream>
#include <sstream>
#include <cstdlib>	// for std::getenv()
#include <sys/stat.h>	// for mkdir()
#include <errno.h>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h>
#include "Calibrator.h"
#include "HandeyeCalibration.h"

#define DEBUG

namespace o2as_easy_handeye
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
Calibrator::Calibrator()
    :_node(),
     _get_sample_list_srv(
      	 _node.advertiseService("get_sample_list",
				&Calibrator::get_sample_list, this)),
     _take_sample_srv(
	 _node.advertiseService("take_sample",
				&Calibrator::take_sample, this)),
     _compute_calibration_srv(
	 _node.advertiseService("compute_calibration",
				&Calibrator::compute_calibration, this)),
     _save_calibration_srv(
	 _node.advertiseService("save_calibration",
				&Calibrator::save_calibration, this)),
     _reset_srv(_node.advertiseService("reset", &Calibrator::reset, this)),
     _listener(),
     _use_dual_quaternion(false),
     _eye_on_hand(true),
     _timeout(5.0)
{
    ROS_INFO_STREAM("initializing calibrator...");

    _node.param<bool>("use_dual_quaternion", _use_dual_quaternion, false);
    _node.param<bool>("eye_on_hand", _eye_on_hand, true);

    if (_eye_on_hand)
    {
	_node.param<std::string>("robot_effector_frame", _eMc.header.frame_id,
				 "tool0");
	_node.param<std::string>("robot_base_frame",	 _wMo.header.frame_id,
				 "base_link");
    }
    else
    {
	_node.param<std::string>("robot_effector_frame", _wMo.header.frame_id,
				 "tool0");
	_node.param<std::string>("robot_base_frame",	 _eMc.header.frame_id,
				 "base_link");
    }

    _node.param<std::string>("tracking_base_frame",   _eMc.child_frame_id,
			     "tracking_origin");
    _node.param<std::string>("tracking_marker_frame", _wMo.child_frame_id,
			     "tracking_target");
}

Calibrator::~Calibrator()
{
}

void
Calibrator::spin()
{
    ros::spin();
}

const std::string&
Calibrator::camera_frame() const
{
    return _eMc.child_frame_id;
}

const std::string&
Calibrator::effector_frame() const
{
    return _eMc.header.frame_id;
}

const std::string&
Calibrator::object_frame() const
{
    return _wMo.child_frame_id;
}

const std::string&
Calibrator::world_frame() const
{
    return _wMo.header.frame_id;
}

bool
Calibrator::get_sample_list(GetSampleList::Request&,
			    GetSampleList::Response& res)
{
    res.success = true;
    res.message = std::to_string(_cMo.size()) + " samples in hand.";
    res.cMo	= _cMo;
    res.wMe	= _wMe;

    ROS_INFO_STREAM("get_sample_list(): " << res.message);

    return true;
}

bool
Calibrator::take_sample(std_srvs::Trigger::Request&,
			std_srvs::Trigger::Response& res)
{
    try
    {
	const auto	now = ros::Time::now();

	_listener.waitForTransform(camera_frame(), object_frame(),
				   now, ros::Duration(_timeout));
	_listener.waitForTransform(world_frame(), effector_frame(),
				   now, ros::Duration(_timeout));

	tf::StampedTransform	cMo, wMe;
	_listener.lookupTransform(camera_frame(), object_frame(),   now, cMo);
	_listener.lookupTransform(world_frame(),  effector_frame(), now, wMe);

	transformMsg_t	msg;
	tf::transformStampedTFToMsg(cMo, msg);
	_cMo.emplace_back(msg);
	tf::transformStampedTFToMsg(wMe, msg);
	_wMe.emplace_back(msg);

	res.success = true;
	res.message = "succeeded.";

	ROS_INFO_STREAM("take_sample(): " << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("take_sample(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::compute_calibration(ComputeCalibration::Request&,
				ComputeCalibration::Response& res)
{
    try
    {
	using transform_t	= TU::Transform<double>;

	ROS_INFO_STREAM("compute_calibration(): computing with "
			<< (_use_dual_quaternion ? "DUAL quaternion"
						 : "SINGLE quaternion")
			<< " algorithm...");

	std::vector<transform_t>	cMo, wMe;
	for (size_t i = 0; i < _cMo.size(); ++i)
	{
	    cMo.emplace_back(_cMo[i].transform);
	    wMe.emplace_back(_wMe[i].transform);
	}

	const auto	eMc = (_use_dual_quaternion ?
			       TU::cameraToEffectorDual(cMo, wMe) :
			       TU::cameraToEffectorSingle(cMo, wMe));
	const auto	wMo = TU::objectToWorld(cMo, wMe, eMc);

	const auto	now = ros::Time::now();
	_eMc.header.stamp = now;
	_eMc.transform	  = eMc;
	_wMo.header.stamp = now;
	_wMo.transform	  = wMo;

	std::ostringstream	sout;
	TU::evaluateAccuracy(sout, cMo, wMe, eMc, wMo);
	res.success = true;
	res.message = sout.str();
	res.eMc	    = _eMc;
	res.wMo	    = _wMo;

	ROS_INFO_STREAM("compute_calibration(): " << res.message);

#ifdef DEBUG
	std::ofstream	out("cMo_wMe_pairs.txt");
	out << cMo.size() << std::endl;
	for (size_t i = 0; i < cMo.size(); ++i)
	    out << cMo[i] << std::endl
		<< wMe[i] << std::endl << std::endl;
	out << sout.str();
#endif
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("compute_calibration(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::save_calibration(std_srvs::Trigger::Request&,
			     std_srvs::Trigger::Response& res)
{
    try
    {
	YAML::Emitter	emitter;
	emitter << YAML::BeginMap;
	emitter << YAML::Key   << "eye_on_hand"
		<< YAML::Value << _eye_on_hand;
	emitter << YAML::Key   << (_eye_on_hand ? "robot_effector_frame"
						: "robot_base_frame")
		<< YAML::Value << effector_frame();
	emitter << YAML::Key   << "tracking_base_frame"
		<< YAML::Value << camera_frame();
	emitter << YAML::Key   << "transformation"
		<< YAML::Value
		<< YAML::Flow
		<< YAML::BeginMap
		<< YAML::Key   << "qw"
		<< YAML::Value << _eMc.transform.rotation.w
		<< YAML::Key   << "qx"
		<< YAML::Value << _eMc.transform.rotation.x
		<< YAML::Key   << "qy"
		<< YAML::Value << _eMc.transform.rotation.y
		<< YAML::Key   << "qz"
		<< YAML::Value << _eMc.transform.rotation.z
		<< YAML::Key   << "x"
		<< YAML::Value << _eMc.transform.translation.x
		<< YAML::Key   << "y"
		<< YAML::Value << _eMc.transform.translation.y
		<< YAML::Key   << "z"
		<< YAML::Value << _eMc.transform.translation.z
		<< YAML::EndMap;
	emitter << YAML::Key   << (_eye_on_hand ? "robot_base_frame"
						: "robot_effector_frame")
		<< YAML::Value << world_frame();
	emitter << YAML::Key   << "tracking_marker_frame"
		<< YAML::Value << object_frame();
	emitter << YAML::Key   << "another_transformation"
		<< YAML::Value
		<< YAML::Flow
		<< YAML::BeginMap
		<< YAML::Key   << "qw"
		<< YAML::Value << _wMo.transform.rotation.w
		<< YAML::Key   << "qx"
		<< YAML::Value << _wMo.transform.rotation.x
		<< YAML::Key   << "qy"
		<< YAML::Value << _wMo.transform.rotation.y
		<< YAML::Key   << "qz"
		<< YAML::Value << _wMo.transform.rotation.z
		<< YAML::Key   << "x"
		<< YAML::Value << _wMo.transform.translation.x
		<< YAML::Key   << "y"
		<< YAML::Value << _wMo.transform.translation.y
		<< YAML::Key   << "z"
		<< YAML::Value << _wMo.transform.translation.z
		<< YAML::EndMap;
	emitter << YAML::EndMap;

	const auto	home = getenv("HOME");
	if (!home)
	    throw std::runtime_error("environment variable HOME is not set.");

	std::string	name = home + std::string("/.ros/easy_handeye");
	struct stat	buf;
	if (stat(name.c_str(), &buf) && mkdir(name.c_str(), S_IRWXU))
	    throw std::runtime_error("cannot create " + name + ": "
						      + strerror(errno));

	name += (ros::this_node::getNamespace() + ".yaml");
	std::ofstream	out(name.c_str());
	if (!out)
	    throw std::runtime_error("cannot open " + name + ": "
						    + strerror(errno));

	out << emitter.c_str() << std::endl;

	res.success = true;
	res.message = "saved in " + name;
	ROS_INFO_STREAM("save_calibration(): " << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("compute_calibration(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    _cMo.clear();
    _wMe.clear();

    ROS_INFO_STREAM("reset(): all samples cleared.");

    return true;
}

}	// namespace o2as_easy_handeye
