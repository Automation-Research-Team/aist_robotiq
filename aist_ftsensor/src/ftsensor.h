/*!
 *  \file	ftsensor.h
 *  \brief	header of a ROS node class for controlling force sensors
 */
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>

namespace aist_ftsensor
{
/************************************************************************
*  class ftsensor							*
************************************************************************/
class ftsensor
{
  private:
    using wrench_t	 = geometry_msgs::WrenchStamped;
    using wrench_p	 = geometry_msgs::WrenchStampedPtr;
    using const_wrench_p = geometry_msgs::WrenchStampedConstPtr;
    using transform_t	 = tf::StampedTransform;

  public:
		ftsensor(const std::string& name)			;
		~ftsensor()						;

    void	run()							;
    double	rate()						const	;

    void	wrench_callback(const const_wrench_p& wrench_msg)       ;

  private:
    ros::NodeHandle		_nh;
    const ros::Publisher	_publisher;
    const ros::Subscriber	_subscriber;
    const tf::TransformListener	_listener;
    std::string			_reference_frame;
    std::string			_sensor_frame;
    double			_rate;
    tf::Vector3			_mg;
    tf::Vector3			_f_offset;
    tf::Vector3			_m_offset;
    tf::Vector3			_r_offset;
};

}	// namespace aist_ftsensor
