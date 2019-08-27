/*!
 *  \file	ftsensor.cpp
 *  \brief	source file for a class for controlling FT300 force sensors
 */
#include "ftsensor.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

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
     _m(0),
     _f0(Eigen::Vector3f::Zero()),
     _m0(Eigen::Vector3f::Zero()),
     _r0(Eigen::Vector3f::Zero()),
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

    try
    {
	YAML::Node yaml_node = YAML::LoadFile(filepath());

	double effector_mass = yaml_node[KEY_EFFECTOR_MASS].as<double>();
	if (effector_mass > 0)
	    _m = effector_mass;
	ROS_INFO_STREAM(KEY_EFFECTOR_MASS << "=" << _m);

	std::vector<double> vec;
	vec = yaml_node[KEY_F_OFFSET].as<std::vector<double> >();
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_f0(i) = vec[i];
	}
	ROS_INFO_STREAM(KEY_F_OFFSET << "\n" << _f0);

	vec.clear();
	vec = yaml_node[KEY_M_OFFSET].as<std::vector<double> >();
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_m0(i) = vec[i];
	}
	ROS_INFO_STREAM(KEY_M_OFFSET << "\n" << _m0);

	vec.clear();
	vec = yaml_node[KEY_R_OFFSET].as<std::vector<double> >();
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_r0(i) = vec[i];
	}
	ROS_INFO_STREAM(KEY_R_OFFSET << "\n" << _r0);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }

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
	Eigen::Vector3f	k(
		T.getBasis()[2].x(), T.getBasis()[2].y(), T.getBasis()[2].z());
	if (_get_sample)
	{
	    take_sample(k, wrench_msg->wrench.force);
	    _get_sample = false;
	}

	wrench_p	wrench(new wrench_t);
	wrench->header = wrench_msg->header;
	wrench->wrench = wrench_msg->wrench;
	wrench->header.frame_id = _sensor_frame;

	Eigen::Vector3f force  = _m*G*k + _f0;
	Eigen::Vector3f torque = _r0.cross(_m*G*k) + _m0;

	wrench->wrench.force.x  = wrench_msg->wrench.force.x  - force(0);
	wrench->wrench.force.y  = wrench_msg->wrench.force.y  - force(1);
	wrench->wrench.force.z  = wrench_msg->wrench.force.z  - force(2);
	wrench->wrench.torque.x = wrench_msg->wrench.torque.x - torque(0);
	wrench->wrench.torque.y = wrench_msg->wrench.torque.y - torque(1);
	wrench->wrench.torque.z = wrench_msg->wrench.torque.z - torque(2);

	_publisher.publish(wrench);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }
}

void
ftsensor::take_sample(const Eigen::Vector3f& k, const geometry_msgs::Vector3& f)
{
#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("k\n" << k);
    showVector3("f", f);
#endif /* __MY_DEBUG__ */

    Eigen::Matrix4f At_A;
    At_A <<     1,     0,     0, -k(0),
                0,     1,     0, -k(1),
                0,     0,     1, -k(2),
            -k(0), -k(1), -k(2),     1;
    _At_A += At_A;
#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("At_A\n" << At_A);
    ROS_INFO_STREAM("_At_A\n" << _At_A);
#endif /* __MY_DEBUG__ */

    Eigen::Vector4f At_b;
    At_b << f.x, f.y, f.z, -(k(0)*f.x + k(1)*f.y + k(2)*f.z);
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
    _f0 << result(0), result(1), result(2);
    _m  = result(3)/G;

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
    try
    {
	YAML::Emitter emitter;
	emitter << YAML::BeginMap;
	emitter << YAML::Key << KEY_EFFECTOR_MASS << YAML::Value << _m;
	emitter << YAML::Key << KEY_F_OFFSET      << YAML::Value
		<< YAML::BeginSeq
		<< _f0.x() << _f0.y() << _f0.z()
		<< YAML::EndSeq;
	emitter << YAML::EndMap;

	std::ofstream f(filepath());
	f << emitter.c_str() << std::endl;

	res.success = true;
	res.message = "save_calibration succeeded.";
	ROS_INFO_STREAM(res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = "save_calibration failed.";
	res.message += err.what();
	ROS_ERROR_STREAM(res.message);
    }

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
