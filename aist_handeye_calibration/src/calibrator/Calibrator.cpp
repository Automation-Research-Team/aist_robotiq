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

namespace aist_handeye_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
Calibrator::Calibrator(const ros::NodeHandle& nh)
    :_nh(nh),
     _get_sample_list_srv(
	 _nh.advertiseService("get_sample_list",
			      &Calibrator::get_sample_list, this)),
     _take_sample_srv(
	 _nh.advertiseService("take_sample", &Calibrator::take_sample, this)),
     _compute_calibration_srv(
	 _nh.advertiseService("compute_calibration",
			      &Calibrator::compute_calibration, this)),
     _save_calibration_srv(
	 _nh.advertiseService("save_calibration",
			      &Calibrator::save_calibration, this)),
     _reset_srv(_nh.advertiseService("reset", &Calibrator::reset, this)),
     _listener(),
     _use_dual_quaternion(false),
     _eye_on_hand(true),
     _timeout(5.0)
{
    ROS_INFO_STREAM("initializing calibrator...");

    _nh.param<bool>("use_dual_quaternion",
		    _use_dual_quaternion, _use_dual_quaternion);
    _nh.param<bool>("eye_on_hand", _eye_on_hand, _eye_on_hand);

    if (_eye_on_hand)
    {
	_nh.param<std::string>("robot_effector_frame",	_eMc.header.frame_id,
			       "tool0");
	_nh.param<std::string>("robot_base_frame",	_wMo.header.frame_id,
			       "base_link");
    }
    else
    {
	_nh.param<std::string>("robot_effector_frame",	_wMo.header.frame_id,
			       "tool0");
	_nh.param<std::string>("robot_base_frame",	_eMc.header.frame_id,
			       "base_link");
    }

    _nh.param<std::string>("camera_frame", _eMc.child_frame_id, "camera_frame");
    _nh.param<std::string>("marker_frame", _wMo.child_frame_id, "marker_frame");
}

Calibrator::~Calibrator()
{
}

void
Calibrator::run()
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

	emitter << YAML::Key   << "parent"
		<< YAML::Value << effector_frame();
	emitter << YAML::Key   << "child"
		<< YAML::Value << camera_frame();
	emitter << YAML::Key   << "transform"
		<< YAML::Value
		<< YAML::Flow
		<< YAML::BeginMap
		<< YAML::Key   << "x"
		<< YAML::Value << _eMc.transform.translation.x
		<< YAML::Key   << "y"
		<< YAML::Value << _eMc.transform.translation.y
		<< YAML::Key   << "z"
		<< YAML::Value << _eMc.transform.translation.z
		<< YAML::Key   << "qx"
		<< YAML::Value << _eMc.transform.rotation.x
		<< YAML::Key   << "qy"
		<< YAML::Value << _eMc.transform.rotation.y
		<< YAML::Key   << "qz"
		<< YAML::Value << _eMc.transform.rotation.z
		<< YAML::Key   << "qw"
		<< YAML::Value << _eMc.transform.rotation.w
		<< YAML::EndMap;

	std::string	calib_file;
	_nh.param<std::string>("calib_file", calib_file,
			       getenv("HOME") + std::string("/.ros/aist_handeye_calibration/calib.yaml"));

	const auto	dir = calib_file.substr(0,
						calib_file.find_last_of('/'));
	struct stat	buf;
	if (stat(dir.c_str(), &buf) && mkdir(dir.c_str(), S_IRWXU))
	    throw std::runtime_error("cannot create " + dir + ": "
						      + strerror(errno));

	std::ofstream	out(calib_file.c_str());
	if (!out)
	    throw std::runtime_error("cannot open " + calib_file + ": "
						    + strerror(errno));

	out << emitter.c_str() << std::endl;

	res.success = true;
	res.message = "saved in " + calib_file;
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

}	// namespace aist_handeye_calibration
