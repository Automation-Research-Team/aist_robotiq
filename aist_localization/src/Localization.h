/*!
 *  \file	Localization.h
 *  \author	Toshio UESHIBA
 *  \brief	ROS wrapper of Photoneo Localization SDK
 */
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <cstdint>
#include <memory>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_localization/localizeAction.h>

#include <PhoLocalization.h>

namespace aist_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
class Localization
{
  private:
    using action_t	= aist_localization::localizeAction;
    using feedback_t	= aist_localization::localizeFeedback;
    using result_t	= aist_localization::localizeResult;
    using goal_cp	= aist_localization::localizeGoalConstPtr;

    using server_t	= actionlib::SimpleActionServer<action_t>;

  public:
		Localization(const std::string& name)			;

    void	run()						  const	;

  private:
    bool	load_scene_cb(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)	;
    void	localize_cb(const goal_cp& goal)		  const	;
    void	preempt_cb()					  const	;
    void	publish_feedback(const pho::sdk::LocalizationPose& locPose,
				 ros::Time time,
				 const std::string& object_frame) const	;
    static std::string
		scene_dir()						;

  private:
    ros::NodeHandle				_nh;

    std::string					_camera_frame;
    std::string					_plcf_dir;

    std::unique_ptr<pho::sdk::PhoLocalization>	_localization;
    pho::sdk::SceneSource			_scene;
    bool					_scene_is_valid;

    const ros::ServiceServer			_load_scene_srv;
    server_t					_localize_srv;
};

}	// namespace aist_localization
#endif	// LOCALIZATION_H
