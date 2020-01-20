/*!
 *  \file	Localization.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS wrapper of Photoneo Localization SDK
 */
#include <ros/package.h>
#include "Localization.h"

namespace aist_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
Localization::Localization(const std::string& name)
    :_nh(name),
     _tfBroadcaster(),
     _camera_frame("camera_frame"),
     _config_dir(""),
     _localization(new pho::sdk::PhoLocalization()),
     _scene(),
     _scene_is_valid(false),
     _load_scene_srv(_nh.advertiseService("load_scene", &load_scene_cb, this)),
     _localize_srv(_nh, "localize_from_ply",
		   boost::bind(&Localization::localize_cb, this, _1), false)
{
    _nh.param("camera_frame", _camera_frame, std::string("camera_frame"));
    _nh.param("config_dir",   _config_dir,
	      ros::package::getPath("aist_localization") + "/config");
    _localize_srv.start();
}

Localization::~Localization()
{
}
    
void
Localization::run()
{
    ros::spin();
}

bool
Localization::load_scene_cb(std_srvs::Trigger::Request&  req,
			    std_srvs::Trigger::Response& res)
{
    try
    {
	const auto	scene_file = scene_dir() + "/scene.ply";
	
	ROS_INFO_STREAM("load_scene: [" << scene_file << "]..,");

	_scene = pho::sdk::SceneSource::File(scene_file);
	_localization->SetSceneSource(_scene);
	_scene_is_valid = true;

	ROS_INFO_STREAM("  succeeded.");
    }
    catch (const std::exception& err)
    {
	_scene_is_valid = false;

	ROS_ERROR_STREAM("  failed: " << err.what());
    }

    res.success = _scene_is_valid;
    res.message = (res.success ? "succeded" : "failed");
    
    return true;
}
    
void
Localization::localize_cb(const goal_cp& goal)
{
    try
    {
	ROS_INFO_STREAM("localize: Executing for config["
			<< goal->config_file << "].");

	if (!_scene_is_valid)
	    throw std::runtime_error("Scene is not set.");

      // Load configuration.
	const auto	config_file = _config_dir + '/' + goal->config_file;
	if (!_localization->LoadLocalizationConfiguration(config_file))
	    throw std::runtime_error("Failed to load configuration["
				     + config_file + "].");
	
      // Set stop criteria.
	_localization->SetStopCriterion(
	    pho::sdk::StopCriterion::Timeout(int(1000 * goal->timeout)));
	_localization->SetStopCriterion(
	    pho::sdk::StopCriterion::NumberOfResults(goal->number_of_poses));

      //
	std::vector<pho::sdk::LocalizationPose>	locPoses;
	
	for (auto queue = _localization->StartAsync();
	     queue.Size() != goal->number_of_poses; )
	{
	    if (_localize_srv.isPreemptRequested() || !ros::ok())
	    {
		ROS_INFO_STREAM("locallize: Preempted.");

		_localize_srv.setPreempted();
		break;
	    }

	    pho::sdk::LocalizationPose	locPose;
	    if (queue.GetNext(locPose, 500))	// Timeout = 500ms
		locPoses.push_back(locPose);
	
	  // Feedback the number of poses currently found.
	    feedback_t	feedback;
	    feedback.number_of_poses = locPoses.size();
	    _localize_srv.publishFeedback(feedback);
	}
	
	_localization->StopAsync();

	std::sort(locPoses.begin(), locPoses.end(),
		  [](const auto& a, const auto& b)
		  { return a.VisibleOverlap > b.VisibleOverlap; });
	
		  
	result_t	result;
	result.header.stamp	= ros::Time::now();
	result.header.frame_id	= _camera_frame;
	for (const auto& locPose : locPoses)
	{
	    result.poses.push_back();
	    result.overlaps.push_back(locPose.VisibleOverlap);
	}
	
	_localize_srv.setSucceeded(result);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("localize_from_ply: " << err.what());
    }
}

std::string
Localization::scene_dir()
{
    const auto	home = getenv("HOME");
    if (!home)
	throw std::runtime_error("Environment variable[HOME] is not set.");

    return home + std::string("/.ros") + ros::this_node::getNamespace();
}
    
}	// namespace aist_localization
