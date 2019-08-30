/*!
 *  \file	ftsensor.h
 *  \brief	header of a ROS node class for controlling force sensors
 */
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>

#include <fstream>
#include <eigen3/Eigen/Dense>


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

    constexpr static float	G = 9.8;
    constexpr static auto&	KEY_EFFECTOR_MASS = "effector_mass";
    constexpr static auto&	KEY_F_OFFSET	   = "f_offset";
    constexpr static auto&	KEY_M_OFFSET	   = "m_offset";
    constexpr static auto&	KEY_R_OFFSET	   = "r_offset";

  public:
    enum InputWrenchType
    {
	SUBSCRIBE = 0,
	SOCKET,
    };

  public:
		ftsensor(const std::string& name,
		    const InputWrenchType iwrench=InputWrenchType::SUBSCRIBE);
		~ftsensor()						;

    void	run()							;
    void	tick()							;
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
    std::string	filepath()					const	;

  private:
    ros::NodeHandle		_nh;
    const InputWrenchType	_iwrench;
    const int			_socket;
    const ros::Publisher	_publisher;
    const ros::Subscriber	_subscriber;
    const tf::TransformListener	_listener;
    std::string			_reference_frame;
    std::string			_sensor_frame;
    double			_rate;
    double			_m;
    Eigen::Matrix<double, 3, 1>	_f0;
    Eigen::Matrix<double, 3, 1>	_m0;
    Eigen::Matrix<double, 3, 1>	_r0;

    const ros::ServiceServer	_take_sample;
    const ros::ServiceServer	_compute_calibration;
    const ros::ServiceServer	_save_calibration;
    bool			_get_sample;
    Eigen::Matrix<double, 4, 4>	_At_A;	    // force
    Eigen::Matrix<double, 4, 1>	_At_b;	    // force
    Eigen::Matrix<double, 6, 6>	_Ct_C;	    // torque
    Eigen::Matrix<double, 6, 1>	_Ct_d;	    // torque

    void	up_socket()						;
    void	down_socket()						;
    bool	connect_socket(u_long hostname, int port)		;

    void	take_sample(const Eigen::Matrix<double, 3, 1>& k,
			    const geometry_msgs::Vector3& f,
			    const geometry_msgs::Vector3& m)		;

#if defined(__MY_DEBUG__) && ( __MY_DEBUG__ > 1 )
  public:
    constexpr static auto& DBG_DUMP_FILE = "/tmp/aist_ftsensor_dbg.dump";
    void	dbg_take_sample(const Eigen::Matrix<double, 3, 1>& k,
			    const geometry_msgs::Vector3& f,
			    const geometry_msgs::Vector3& m)
    {
	_to_dump = false;
	take_sample(k, f, m);
    }
  private:
    bool _to_dump = true;
#endif /* __MY_DEBUG__ */
};

}	// namespace aist_ftsensor
