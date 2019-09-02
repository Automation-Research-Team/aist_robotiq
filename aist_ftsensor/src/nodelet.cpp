/*!
 *  \file	nodelet.cpp
 *  \brief	wrench publisher for force-torque sensors
 */
#include "ftsensor.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace aist_ftsensor
{
/************************************************************************
*  class ftsensor_nodelet							*
************************************************************************/
class ftsensor_nodelet : public nodelet::Nodelet
{
  public:
    ftsensor_nodelet()							{}

    virtual void	onInit()					;
    void		timer_callback(const ros::TimerEvent&)		;

  private:
    ros::NodeHandle		_nh;
    boost::shared_ptr<ftsensor>	_node;
    ros::Timer			_timer;
};

void
ftsensor_nodelet::onInit()
{
    NODELET_INFO("aist_ftsensor::ftsensor_nodelet::onInit()");

    _nh = getNodeHandle();
    _node.reset(new ftsensor(getName()));
    _timer = _nh.createTimer(ros::Duration(1.0/_node->rate()),
			     &ftsensor_nodelet::timer_callback, this);
}

void
ftsensor_nodelet::timer_callback(const ros::TimerEvent&)
{
    _node->tick();
}

}

PLUGINLIB_EXPORT_CLASS(aist_ftsensor::ftsensor_nodelet, nodelet::Nodelet);
