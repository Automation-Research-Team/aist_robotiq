/*!
 *  \file	Localization.cpp
 *  \author	Toshio UESHIBA
 *  \brief	Thin wraper of Photoneo Localization SDK
 */
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
     _localization(new pho::sdk::PhoLocalization()),
     _scene(),
     _scene_is_valid(false),
     _load_scene_srv(_nh.advertiseService("load_scene", &load_scene_cb, this)),
     _localize_srv(_nh, "localize_from_ply",
		   boost::bind(&Localization::localize_cb, this, _1), false)
{
    _nh.param("camera_frame", _camera_frame, std::string("camera_frame"));
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
Localization::load_scene_cb(LoadScene::Request& req, LoadScene::Response& res)
{
    try
    {
	ROS_INFO_STREAM("load_scene: [" << req.scene_file << "]..,");

	_scene = pho::sdk::SceneSource::File(req.scene_file);
	_localization->SetSceneSource(_scene);
	_scene_is_valid = true;

	ROS_INFO_STREAM("  succeeded.");
    }
    catch (const pho::sdk::PhoLocalizationException& err)
    {
	_scene_is_valid = false;

	ROS_ERROR_STREAM("  failed: " << err.what());
    }

    res.success = _scene_is_valid;
    
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
	if (!_localization->LoadLocalizationConfiguration(goal->config_file))
	    throw std::runtime_error("Failed to load configuration["
				     + goal->config_file + "].");
	
      // Set stop criteria.
	_localization->SetStopCriterion(
	    pho::sdk::StopCriterion::Timeout(int(1000 * goal->timeout)));
	_localization->SetStopCriterion(
	    pho::sdk::StopCriterion::NumberOfResults(goal->number_of_poses));

	auto	queue = _localization->StartAsync();
	bool	success = true;
	
	for (ros::Rate rate(1); ; rate.sleep())
	{
	    if (_localize_srv.isPreemptRequested() || !ros::ok())
	    {
		ROS_INFO_STREAM("locallize: Preempted.");

		_localize_srv.setPreempted();
		success = false;
		break;
	    }

	    pho::sdk::TransformationMatrix4x4	transform;
	    if (queue.GetNext(transform, ))
	    {
	    }
	    
	
	  // Feedback the number of poses currently found.
	  //_feedback.number_of_poses =;
	    _localize_srv.publishFeedback(_feedback);

	}
	
	_localization->StopAsync();

	if (success)
	{
	  //_result.;

	    _localize_srv.setSucceeded(_result);
	}
    }
    catch (const pho::sdk::PhoLocalizationException& err)
    {
	ROS_ERROR_STREAM("localize_from_ply: " << err.what());
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("localize_from_ply: " << err.what());
    }
}

}	// namespace aist_localization
