/*!
  \file		Calibrator.h
  \brief	Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <o2as_easy_handeye/GetSampleList.h>
#include <o2as_easy_handeye/ComputeCalibration.h>

namespace o2as_easy_handeye
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator
{
  public:
    using transformMsg_t	= geometry_msgs::TransformStamped;

  public:
		Calibrator()						;
		~Calibrator()						;

    void	spin();

  private:
    const std::string&	camera_frame()				const	;
    const std::string&	effector_frame()			const	;
    const std::string&	object_frame()				const	;
    const std::string&	world_frame()				const	;

    bool	get_sample_list(GetSampleList::Request&,
				GetSampleList::Response& res)		;
    bool	take_sample(std_srvs::Trigger::Request&,
			    std_srvs::Trigger::Response& res)		;
    bool	compute_calibration(ComputeCalibration::Request&,
				    ComputeCalibration::Response& res)	;
    bool	save_calibration(std_srvs::Trigger::Request&,
				 std_srvs::Trigger::Response& res)	;
    bool	reset(std_srvs::Empty::Request&,
		      std_srvs::Empty::Response&)			;

  private:
    ros::NodeHandle		_node;

    const ros::ServiceServer	_get_sample_list_srv;
    const ros::ServiceServer	_take_sample_srv;
    const ros::ServiceServer	_compute_calibration_srv;
    const ros::ServiceServer	_save_calibration_srv;
    const ros::ServiceServer	_reset_srv;

    const tf::TransformListener	_listener;

    std::vector<transformMsg_t>	_cMo;	//!< in:  camera <- object   transform
    std::vector<transformMsg_t>	_wMe;	//!< in:  world  <- effector transform
    transformMsg_t		_eMc;	//!< out: effector <- camera transform
    transformMsg_t		_wMo;	//!< out: world    <- object transform

    bool			_use_dual_quaternion;

    bool			_eye_on_hand;
    double			_timeout;
};
}	// namespace visp_hnad2eye_calibration
#endif	// !CALIBRATOR_H
