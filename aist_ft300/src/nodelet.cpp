/*!
 *  \file	nodelet.cpp
 *  \brief	wrench publisher for FT300 force sensors
 */
#include "ft300.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace aist_ft300
{
/************************************************************************
*  class ft300_nodelet							*
************************************************************************/
class ft300_nodelet : public nodelet::Nodelet
{
  public:
    ft300_nodelet()							{}

    virtual void	onInit()					;
    void		timer_callback(const ros::TimerEvent&)		;

  private:
    ros::NodeHandle		_nh;
    boost::shared_ptr<ft300>	_node;
    ros::Timer			_timer;
};

void
ft300_nodelet::onInit()
{
    NODELET_INFO("aist_ft300::ft300_nodelet::onInit()");

    _nh = getNodeHandle();
    _node.reset(new ft300(getName()));
    _timer = _nh.createTimer(ros::Duration(1.0/_node->rate()),
			     &ft300_nodelet::timer_callback, this);
}

void
ft300_nodelet::timer_callback(const ros::TimerEvent&)
{
    _node->tick();
}

}

PLUGINLIB_EXPORT_CLASS(aist_ft300::ft300_nodelet, nodelet::Nodelet);
