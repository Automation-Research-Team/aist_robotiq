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

  private:
    ros::NodeHandle		_nh;
    boost::shared_ptr<Detector>	_node;
};

void
DetectorNodelet::onInit()
{
    NODELET_INFO("o2as_aruco_ros::DetectorNodelet::onInit()");
    _nh = getNodeHandle();
    _node.reset(new Detector(getName()));
}

}	// namespace o2as_aruco_ros

PLUGINLIB_EXPORT_CLASS(o2as_aruco_ros::DetectorNodelet, nodelet::Nodelet);
