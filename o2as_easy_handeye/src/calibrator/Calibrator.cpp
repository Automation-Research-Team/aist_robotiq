/*!
  \file	 Calibrator.cpp
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/
#include <fstream>
#include <sstream>
#include <tf/transform_datatypes.h>
#include "Calibrator.h"
#include "HandeyeCalibration.h"

#define USE_AIST_CALIBRATION
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
     _timeout(10.0)
{
    ROS_INFO_STREAM("initializing calibrator...");

    bool	eye_on_hand;
    _node.param<bool>("eye_on_hand", eye_on_hand, true);

    if (eye_on_hand)
    {
	_node.param<std::string>("effector_frame", _eMc.header.frame_id,
				 "tool0");
	_node.param<std::string>("world_frame",	   _wMo.header.frame_id,
				 "base_link");
    }
    else
    {
	_node.param<std::string>("effector_frame", _wMo.header.frame_id,
				 "tool0");
	_node.param<std::string>("world_frame",	   _eMc.header.frame_id,
				 "base_link");
    }

    _node.param<std::string>("camera_frame", _eMc.child_frame_id,
			     "tracking_origin");
    _node.param<std::string>("object_frame", _wMo.child_frame_id,
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
Calibrator::get_sample_list(GetSampleList::Request&  req,
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
Calibrator::take_sample(std_srvs::Trigger::Request&  req,
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
Calibrator::compute_calibration(ComputeCalibration::Request&  req,
				ComputeCalibration::Response& res)
{
    if (_cMo.size() != _wMe.size() || _cMo.size() < 2)
    {
	res.success = false;
	res.message = "transformation vectors have different sizes or contain too few elements";

	ROS_ERROR_STREAM("compute_calibration(): " << res.message);
    }
    else
    {
	using transform_t	= TU::Transform<double>;

	ROS_INFO_STREAM("compute_calibration(): computing...");

	std::vector<transform_t>	cMo, wMe;
	for (size_t i = 0; i < _cMo.size(); ++i)
	{
	    cMo.emplace_back(_cMo[i].transform);
	    wMe.emplace_back(_wMe[i].transform);
	}

	const auto	eMc = TU::calibrationDual(cMo, wMe);
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

    return res.success;
}

bool
Calibrator::save_calibration(std_srvs::Trigger::Request&  req,
			     std_srvs::Trigger::Response& res)
{
    res.success = true;
    ROS_INFO_STREAM("save_calibration(): " << res.message);

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
