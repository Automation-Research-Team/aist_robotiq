/*!
 *  \file	ftsensor.h
 *  \brief	header of a ROS node class for controlling force sensors
 */
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>

#include <fstream>
#include <Eigen/Dense>


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

    void	wrench_callback(const const_wrench_p& wrench_msg)	;
    bool	take_sample_callback(std_srvs::Trigger::Request  &req,
				     std_srvs::Trigger::Response &res)	;
    bool	compute_calibration_callback(
			std_srvs::Trigger::Request  &req,
			std_srvs::Trigger::Response &res)		;
    bool	save_calibration_callback(
			std_srvs::Trigger::Request  &req,
			std_srvs::Trigger::Response &res)		;

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

    const ros::ServiceServer	_take_sample;
    const ros::ServiceServer	_compute_calibration;
    const ros::ServiceServer	_save_calibration;
    bool			_get_sample;
    Eigen::Matrix4f		_Atranspose_A;
    Eigen::Vector4f		_Atranspose_b;
    Eigen::Vector4f		_calibration_result;

    void	take_sample(const tf::Matrix3x3& Rt,
			    const geometry_msgs::Vector3& f)		;

#ifdef __MY_DEBUG__
    void showMatrix3x3(const std::string& msg, const tf::Matrix3x3& m)
    {
	ROS_INFO("%s [%f %f %f][%f %f %f][%f %f %f]",
		msg.c_str(),
		m[0].x(), m[0].y(), m[0].z(),
		m[1].x(), m[1].y(), m[1].z(),
		m[2].x(), m[2].y(), m[2].z());
    }
    void showVector3(const std::string& msg, const geometry_msgs::Vector3& v)
    {
	ROS_INFO("%s [%f %f %f]", msg.c_str(), v.x, v.y, v.z);
    }
#endif /* __MY_DEBUG__ */

};

}	// namespace aist_ftsensor
