/*!
 *  \file	ftsensor.cpp
 *  \brief	source file for a class for controlling FT300 force sensors
 */
#include "ftsensor.h"
#include <ros/package.h>

namespace aist_ftsensor
{
/************************************************************************
*  class ftsensor							*
************************************************************************/
ftsensor::ftsensor(const std::string& name)
    :_nh(name),
     _publisher(_nh.advertise<wrench_t>("wrench", 100)),
     _subscriber(_nh.subscribe("/wrench_in", 100,
			       &ftsensor::wrench_callback, this)),
     _take_sample(_nh.advertiseService("take_sample",
				       &ftsensor::take_sample_callback, this)),
     _compute_calibration(_nh.advertiseService(
			      "compute_calibration",
			      &ftsensor::compute_calibration_callback, this)),
     _save_calibration(_nh.advertiseService(
			   "save_calibration",
			   &ftsensor::save_calibration_callback, this)),
     _listener(),
     _reference_frame("workspace_center"),
     _sensor_frame("ftsensor_wrench_link"),
     _rate(100),
     _mg(0, 0, 0),
     _f0(0, 0, 0),
     _m0(0, 0, 0),
     _r0(0, 0, 0),
     _get_sample(false),
     _At_A(Eigen::Matrix4f::Zero()),
     _At_b(Eigen::Vector4f::Zero())
{
    _nh.param<std::string>("reference_frame", _reference_frame,
			   "workspace_center");
    _nh.param<std::string>("sensor_frame", _sensor_frame,
			   "ftsensor_wrench_link");
    _nh.param<double>("rate", _rate, 100);

    ROS_INFO_STREAM("reference_frame=" << _reference_frame <<
		    ", sensor_frame=" << _sensor_frame << ", rate=" << _rate);

    double	effector_mass;
    _nh.param<double>("effector_mass", effector_mass, 0);
    if (effector_mass > 0)
	_mg = tf::Vector3(0, 0, -G * effector_mass);
    ROS_INFO_STREAM("mg=[" << _mg.x() << "," << _mg.y() << "," << _mg.z()
		    << ']');

    std::vector<double> vec;
    if (_nh.getParam("f_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_f0.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("f_offset[" << _f0.x() << "," <<
		    _f0.y() << "," << _f0.z() << "]");

    vec.clear();
    if (_nh.getParam("m_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_m0.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("m_offset[" << _m0.x() << "," <<
		    _m0.y() << "," << _m0.z() << "]");

    vec.clear();
    if (_nh.getParam("r_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_r0.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("r_offset[" << _r0.x() << "," <<
		    _r0.y() << "," << _r0.z() << "]");

    ROS_INFO_STREAM("aist_ftsensor started.");
}

ftsensor::~ftsensor()
{
}

void
ftsensor::run()
{
    ros::Rate	rate(_rate);

    while (ros::ok())
    {
	ros::spinOnce();
	rate.sleep();
    }
}

void
ftsensor::wrench_callback(const const_wrench_p& wrench_msg)
{
    // ROS_INFO("#Callback# sec - %d", wrench_msg->header.stamp.sec);

    try
    {
	_listener.waitForTransform(_sensor_frame, _reference_frame,
				   wrench_msg->header.stamp, ros::Duration(10));
	transform_t	T;
	_listener.lookupTransform(_sensor_frame, _reference_frame,
	 			  wrench_msg->header.stamp, T);

	if (_get_sample)
	{
	    take_sample(T.getBasis(), wrench_msg->wrench.force);
	    _get_sample = false;
	}

	wrench_p	wrench(new wrench_t);
	wrench->header = wrench_msg->header;
	wrench->wrench = wrench_msg->wrench;
	wrench->header.frame_id = _sensor_frame;

	tf::Matrix3x3 Rt = T.getBasis();
	tf::Vector3   force  = Rt * _mg + _f0;
	tf::Vector3   torque = _r0.cross(Rt * _mg) + _m0;

	wrench->wrench.force.x  = wrench_msg->wrench.force.x  - force.x();
	wrench->wrench.force.y  = wrench_msg->wrench.force.y  - force.y();
	wrench->wrench.force.z  = wrench_msg->wrench.force.z  - force.z();
	wrench->wrench.torque.x = wrench_msg->wrench.torque.x - torque.x();
	wrench->wrench.torque.y = wrench_msg->wrench.torque.y - torque.y();
	wrench->wrench.torque.z = wrench_msg->wrench.torque.z - torque.z();

	_publisher.publish(wrench);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }
}

void
ftsensor::take_sample(const tf::Matrix3x3& Rt, const geometry_msgs::Vector3& f)
{
#ifdef __MY_DEBUG__
    showMatrix3x3("Rt", Rt);
    showVector3("f", f);
#endif /* __MY_DEBUG__ */

    Eigen::Matrix4f At_A;
    At_A <<           1,          0,          0, -Rt[2].x(),
                      0,          1,          0, -Rt[2].y(),
                      0,          0,          1, -Rt[2].z(),
             -Rt[2].x(), -Rt[2].y(), -Rt[2].z(),          1;
    _At_A += At_A;
#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("At_A\n" << At_A);
    ROS_INFO_STREAM("_At_A\n" << _At_A);
#endif /* __MY_DEBUG__ */

    Eigen::Vector4f At_b;
    At_b << f.x, f.y, f.z, -(Rt[2].x()*f.x + Rt[2].y()*f.y + Rt[2].z()*f.z);
    _At_b += At_b;
#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("At_b\n" <<  At_b);
    ROS_INFO_STREAM("_At_b\n" << _At_b);
#endif /* __MY_DEBUG__ */
}

bool
ftsensor::take_sample_callback(std_srvs::Trigger::Request  &req,
			       std_srvs::Trigger::Response &res)
{
    _get_sample = true;
    res.success = true;
    res.message = "take_sample succeeded.";
    ROS_INFO_STREAM(res.message);

    return true;
}

bool
ftsensor::compute_calibration_callback(std_srvs::Trigger::Request  &req,
				       std_srvs::Trigger::Response &res)
{
    const auto	result = _At_A.inverse() * _At_b;
#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("_At_A\n" << _At_A);
    ROS_INFO_STREAM("(_At_A)inverse\n" << _At_A.inverse());
    ROS_INFO_STREAM("_At_b\n" << _At_b);
    ROS_INFO_STREAM("_calibration_result\n" << result);
#endif /* __MY_DEBUG__ */
    _f0 = tf::Vector3(result(0), result(1), result(2));
    _mg = tf::Vector3(0, 0, result(3));

    _At_A = Eigen::Matrix4f::Zero();
    _At_b = Eigen::Vector4f::Zero();

    res.success = true;
    res.message = "compute_calibration succeeded.";
    ROS_INFO_STREAM(res.message);

    return true;
}

bool
ftsensor::save_calibration_callback(std_srvs::Trigger::Request  &req,
				    std_srvs::Trigger::Response &res)
{
    std::ofstream f(filepath());
    f << "f_offset:\n" <<
	"    " << _f0.x() << "\n"
	"    " << _f0.y() << "\n"
	"    " << _f0.z() << "\n";
    f << "effector_mass: " << _mg.z()/G << "\n";

    res.success = true;
    res.message = "save_calibration succeeded.";
    ROS_INFO_STREAM(res.message);

    return true;
}

std::string
ftsensor::filepath() const
{
    return ros::package::getPath("aist_ftsensor")
	 + "/config/" + _nh.getNamespace() + ".yaml";
}

double
ftsensor::rate() const
{
    return _rate;
}

}	// namespace aist_ftsensor
