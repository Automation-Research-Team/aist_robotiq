/*!
 *  \file	nodelet.cpp
 */
#include "Detector.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace o2as_aruco_ros
{
/************************************************************************
*  class DetectorNodelet						*
************************************************************************/
class DetectorNodelet : public nodelet::Nodelet
{
  public:
			DetectorNodelet()				{}

    virtual void	onInit()					;
    void		timer_callback(const ros::TimerEvent&)		;

  private:
    ros::NodeHandle		_nh;
    boost::shared_ptr<Detector>	_node;
    ros::Timer			_timer;
};

void
DetectorNodelet::onInit()
{
    NODELET_INFO("o2as_aruco_ros::DetectorNodelet::onInit()");
    _nh = getNodeHandle();
    _node.reset(new Detector(getName()));
    _timer = _nh.createTimer(ros::Duration(1.0/_node->rate()),
    			     &DetectorNodelet::timer_callback, this);
}

void
DetectorNodelet::timer_callback(const ros::TimerEvent&)
{
    _node->tick();
}

}	// namespace o2as_aruco_ros

PLUGINLIB_EXPORT_CLASS(o2as_aruco_ros::DetectorNodelet, nodelet::Nodelet);
