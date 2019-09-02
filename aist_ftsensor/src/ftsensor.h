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
  public:
    enum Input
    {
	TOPIC = 0,
	SOCKET,
    };

  private:
    using wrench_t		= geometry_msgs::WrenchStamped;
    using wrench_p		= geometry_msgs::WrenchStampedPtr;
    using const_wrench_p	= geometry_msgs::WrenchStampedConstPtr;
    using transform_t		= tf::StampedTransform;
    using vector3_t		= Eigen::Matrix<double, 3, 1>;
    using vector4_t		= Eigen::Matrix<double, 4, 1>;
    using matrix44_t		= Eigen::Matrix<double, 4, 4>;
    using vector6_t		= Eigen::Matrix<double, 6, 1>;
    using matrix66_t		= Eigen::Matrix<double, 6, 6>;
    
    constexpr static double	G = 9.8;
    constexpr static auto&	KEY_EFFECTOR_MASS = "effector_mass";
    constexpr static auto&	KEY_FORCE_OFFSET  = "force_offset";
    constexpr static auto&	KEY_TORQUE_OFFSET = "torque_offset";
    constexpr static auto&	KEY_MASS_CENTER	  = "mass_center";

  public:
		ftsensor(const std::string& name,
			 const Input input=Input::TOPIC)		;
		~ftsensor()						;

    void	run()							;
    void	tick()							;
    double	rate()						const	;
    bool	take_sample_callback(std_srvs::Trigger::Request&  req,
				     std_srvs::Trigger::Response& res)	;
    bool	compute_calibration_callback(
			std_srvs::Trigger::Request&  req,
			std_srvs::Trigger::Response& res)		;
    bool	save_calibration_callback(
			std_srvs::Trigger::Request&  req,
			std_srvs::Trigger::Response& res)		;

  private:
    void	up_socket()						;
    void	down_socket()						;
    bool	connect_socket(u_long hostname, int port)		;
    void	wrench_callback(const const_wrench_p& wrench)		;
    void	take_sample(const vector3_t& k,
			    const geometry_msgs::Vector3& f,
			    const geometry_msgs::Vector3& m)		;
    std::string	filepath()					const	;
    
  private:
    ros::NodeHandle		_nh;
    const Input			_input;
    const int			_socket;
    const ros::Subscriber	_subscriber;
    const ros::Publisher	_publisher_org;
    const ros::Publisher	_publisher_fixed;
    const tf::TransformListener	_listener;
    std::string			_reference_frame;
    std::string			_sensor_frame;
    double			_rate;
    double			_m;		// effector mass
    vector3_t			_f0;		// force offset
    vector3_t			_m0;		// torque offset
    vector3_t			_r0;		// mass center

    const ros::ServiceServer	_take_sample;
    const ros::ServiceServer	_compute_calibration;
    const ros::ServiceServer	_save_calibration;
    bool			_get_sample;
    matrix44_t			_At_A;		// force
    vector4_t			_At_b;		// force
    matrix66_t			_Ct_C;		// torque
    vector6_t			_Ct_d;		// torque

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
