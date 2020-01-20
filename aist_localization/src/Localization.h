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
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_localization/LocalizeAction.h>

#include <PhoLocalization.h>

namespace aist_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
class Localization
{
  private:
    using action_t	= aist_localization::LocalizeAction;
    using feedback_t	= aist_localization::LocalizeFeedback;
    using result_t	= aist_localization::LocalizeResult;
    using goal_cp	= aist_localization::LocalizeGoalConstPtr;
    
    using server_t	= actionlib::SimpleActionServer<action_t>;
						 
  public:
		Localization(const std::string& name)			;
		~Localization()						;
    
    void	run()							;

  private:
    bool	load_scene_cb(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)		;
    void	localize_cb(const goal_cp& goal)			;
    static std::string
		scene_dir()						;
    
  private:
    ros::NodeHandle				_nh;

    tf::TransformBroadcaster			_tfBroadcaster;
    std::string					_camera_frame;
    std::string					_config_dir;
    
    std::unique_ptr<pho::sdk::PhoLocalization>	_localization;
    pho::sdk::SceneSource			_scene;
    bool					_scene_is_valid;
    
    const ros::ServiceServer			_load_scene_srv;
    server_t					_localize_srv;
};

}	// namespace aist_localization
#endif	// LOCALIZATION_H
