/*!
 *  \file	ft300.h
 *  \brief	header file for a class for controlling FT300 force sensors
 */
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

namespace aist_ft300
{
/************************************************************************
*  class ft300								*
************************************************************************/
class ft300
{
  private:
    using wrench_t = geometry_msgs::WrenchStamped;
    using wrench_p = geometry_msgs::WrenchStampedPtr;

  public:
		ft300(const std::string& name)				;
		~ft300()						;

    void	run()							;
    void	tick()							;
    double	rate()						const	;

  private:
    bool	connect_socket(u_long hostname, int port)		;

  private:
    ros::NodeHandle		_nh;
    const int			_socket;
    const ros::Publisher	_publisher;
    std::string			_frame;
    double			_rate;
};

}	// namespace aist_ft300
