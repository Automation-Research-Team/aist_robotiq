/*!
 *  \file	ftsensor.h
 *  \brief	header file for a class for controlling FT300 force sensors
 */
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

namespace aist_ftsensor
{
/************************************************************************
*  class ftsensor								*
************************************************************************/
class ftsensor
{
  private:
    using wrench_t = geometry_msgs::WrenchStamped;
    using wrench_p = geometry_msgs::WrenchStampedPtr;

  public:
		ftsensor(const std::string& name)			;
		~ftsensor()						;

    void	run()							;
    double	rate()						const	;

    void	wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg);

  private:
    ros::NodeHandle	_nh;
    ros::Publisher	_publisher;
    ros::Subscriber	_subscriber;
    std::string		_frame;
    double		_rate;
};

}	// namespace aist_ftsensor
