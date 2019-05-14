/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Calibrator node
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file calibrator.cpp
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/
#include <fstream>
#include "Calibrator.h"
#include "HandeyeCalibration.h"
#include "names.h"

#define USE_AIST_CALIBRATION
#define DEBUG

namespace visp_hand2eye_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
Calibrator::Calibrator()
    :_node(),
     _computeEffectorCamera(
	 _node.advertiseService(compute_effector_camera_service,
				&Calibrator::computeEffectorCameraCallback,
				this)),
     _computeEffectorCameraQuick(
	 _node.advertiseService(compute_effector_camera_quick_service,
				&Calibrator::computeEffectorCameraQuickCallback,
				this)),
     _reset(_node.advertiseService(reset_service, &Calibrator::resetCallback,
				   this)),
     _check_inputs(ros::NodeHandle(), ros::this_node::getName()),
     _queue_size(1000)
{
    ROS_INFO("o2as_easy_handeye_calibrator: initializing calibrator...");

    ros::V_string	topics;
    topics.push_back(camera_object_topic);
    topics.push_back(world_effector_topic);
    _check_inputs.start(topics, 60.0);
    if (!ros::ok())
      return;

  //define subscribers
    _cMo_subscriber = _node.subscribe(camera_object_topic, _queue_size,
				      &Calibrator::cameraObjectCallback, this);
    _wMe_subscriber = _node.subscribe(world_effector_topic, _queue_size,
				      &Calibrator::worldEffectorCallback, this);
}

Calibrator::~Calibrator()
{
}

void
Calibrator::spin()
{
    ros::spin();
}

void
Calibrator::cameraObjectCallback(
    const geometry_msgs::Transform::ConstPtr& trans)
{
    ROS_DEBUG("o2as_easy_handeye_calibrator: new cMo: [%f,%f,%f -- %f,%f,%f,%f]",
	      trans->translation.x, trans->translation.y, trans->translation.z,
	      trans->rotation.x, trans->rotation.y, trans->rotation.z,
	      trans->rotation.w);

    _cMo.push_back(transform_t(*trans));
}

void
Calibrator::worldEffectorCallback(
    const geometry_msgs::Transform::ConstPtr& trans)
{
    ROS_DEBUG("o2as_easy_handeye_calibrator: new wMe: [%f,%f,%f -- %f,%f,%f,%f]",
	      trans->translation.x, trans->translation.y, trans->translation.z,
	      trans->rotation.x, trans->rotation.y, trans->rotation.z,
	      trans->rotation.w);

    _wMe.push_back(transform_t(*trans));
}

bool
Calibrator::computeEffectorCameraCallback(
    compute_effector_camera::Request&  req,
    compute_effector_camera::Response& res)
{
    if (_cMo.size() != _wMe.size() || _wMe.size() < 2)
    {
      ROS_ERROR("o2as_easy_handeye_calibrator: transformation vectors have different sizes or contain too few elements");
      return false;
    }

    ROS_INFO("o2as_easy_handeye_calibrator: computing %d values...",
	     (int)_wMe.size());

    res.effector_camera = TU::calibrationDual(_cMo, _wMe);
    return true;
}

bool
Calibrator::computeEffectorCameraQuickCallback(
    compute_effector_camera_quick::Request&  req,
    compute_effector_camera_quick::Response& res)
{
    const auto	camera_object  = req.camera_object;
    const auto	world_effector = req.world_effector;

    if (camera_object.transforms.size() != world_effector.transforms.size() ||
	world_effector.transforms.size() < 2)
    {
	ROS_ERROR("o2as_easy_handeye_calibrator: transformation vectors have different sizes or contain too few elements");
	return false;
    }

    ROS_INFO("o2as_easy_handeye_calibrator: computing...");

    std::vector<transform_t>	cMo;
    std::vector<transform_t>	wMe;
    for (size_t i = 0; i < camera_object.transforms.size(); i++)
    {
	cMo.push_back(transform_t(camera_object.transforms[i]));
	wMe.push_back(transform_t(world_effector.transforms[i]));
    }

    res.effector_camera = TU::calibrationDual(cMo, wMe);

#ifdef DEBUG
    transform_t		eMc(res.effector_camera);
    std::ofstream	out("cMo_wMe_pairs.txt");
    out << cMo.size() << std::endl;
    for (size_t n = 0; n < cMo.size(); ++n)
	out << cMo[n] << std::endl
	    << wMe[n] << std::endl << std::endl;

    const auto	wMo = TU::objectToWorld(cMo, wMe, eMc);
    TU::evaluateAccuracy(out, cMo, wMe, eMc, wMo);
#endif
    return true;
}

bool
Calibrator::resetCallback(reset::Request& req, reset::Response& res)
{
    ROS_INFO("o2as_easy_handeye_calibrator: reseting...");
    _cMo.clear();
    _wMe.clear();

    return true;
}

}	// namespace visp_hand2eye_calibration

