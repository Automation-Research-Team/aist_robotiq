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
     _get_sample(false),
     _m(0),
     _f0(Eigen::Matrix<double, 3, 1>::Zero()),
     _m0(Eigen::Matrix<double, 3, 1>::Zero()),
     _r0(Eigen::Matrix<double, 3, 1>::Zero()),
     _At_A(Eigen::Matrix<double, 4, 4>::Zero()),
     _At_b(Eigen::Matrix<double, 4, 1>::Zero()),
     _Ct_C(Eigen::Matrix<double, 6, 6>::Zero()),
     _Ct_d(Eigen::Matrix<double, 6, 1>::Zero())
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
	const auto	colz = T.getBasis().getColumn(2);
	Eigen::Matrix<double, 3, 1> k;
	k << colz.x(), colz.y(), colz.z();
	if (_get_sample)
	{
	    take_sample(k, wrench_msg->wrench.force, wrench_msg->wrench.torque);
	    _get_sample = false;
	}

	wrench_p	wrench(new wrench_t);
	wrench->header = wrench_msg->header;
	wrench->wrench = wrench_msg->wrench;
	wrench->header.frame_id = _sensor_frame;

	Eigen::Matrix<double, 3, 1> force  = -_m*G*k + _f0;
	Eigen::Matrix<double, 3, 1> torque = -_r0.cross(_m*G*k) + _m0;

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
ftsensor::take_sample(const Eigen::Matrix<double, 3, 1>& k,
		      const geometry_msgs::Vector3& f,
		      const geometry_msgs::Vector3& m)
{
    // force
    Eigen::Matrix<double, 4, 4> At_A;
    At_A <<    1,    0,    0, k(0),
               0,    1,    0, k(1),
               0,    0,    1, k(2),
            k(0), k(1), k(2),    1;
    _At_A += At_A;

    Eigen::Matrix<double, 4, 1> At_b;
    At_b << f.x, f.y, f.z, (k(0)*f.x + k(1)*f.y + k(2)*f.z);
    _At_b += At_b;

    // torque
    Eigen::Matrix<double, 3, 3> Ka; // antisymmetric matrix of k
    Ka <<     0, -k(2),  k(1),
           k(2),     0, -k(0),
          -k(1),  k(0),     0;
    Eigen::Matrix<double, 3, 3> Kat = Ka.transpose();
    Eigen::Matrix<double, 3, 3> Kat_Ka = Kat * Ka;

    Eigen::Matrix<double, 6, 6> Ct_C;
    Ct_C <<
	       1,        0,        0,    Ka(0, 0),    Ka(0, 1),    Ka(0, 2),
	       0,        1,        0,    Ka(1, 0),    Ka(1, 1),    Ka(1, 1),
	       0,        0,        1,    Ka(2, 0),    Ka(2, 1),    Ka(2, 2),
	Kat(0,0), Kat(0,1), Kat(0,2), Kat_Ka(0,0), Kat_Ka(0,1), Kat_Ka(0,2),
	Kat(1,0), Kat(1,1), Kat(1,2), Kat_Ka(1,0), Kat_Ka(1,1), Kat_Ka(1,2),
	Kat(2,0), Kat(2,1), Kat(2,2), Kat_Ka(2,0), Kat_Ka(2,1), Kat_Ka(2,2);
    _Ct_C += Ct_C;

    Eigen::Matrix<double, 6, 1> Ct_d;
    Ct_d << m.x, m.y, m.z,
	    k(2)*m.y - k(1)*m.z, k(0)*m.z - k(2)*m.x, k(1)*m.x - k(0)*m.y;
    _Ct_d += Ct_d;

#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("k\n" << k);
    ROS_INFO_STREAM("f\n" << f.x << " " << f.y << " " << f.z);
    ROS_INFO_STREAM("m\n" << m.x << " " << m.y << " " << m.z);
    ROS_INFO_STREAM(" At_A\n" <<  At_A);
    ROS_INFO_STREAM("_At_A\n" << _At_A);
    ROS_INFO_STREAM(" At_b\n" <<  At_b);
    ROS_INFO_STREAM("_At_b\n" << _At_b);
    // ROS_INFO_STREAM(" Ct_C\n" <<  Ct_C);
    // ROS_INFO_STREAM("_Ct_C\n" << _Ct_C);
    // ROS_INFO_STREAM(" Ct_d\n" <<  Ct_d);
    // ROS_INFO_STREAM("_Ct_d\n" << _Ct_d);

#if __MY_DEBUG__ > 1
    if (_to_dump)
    {
	try
	{
	    std::ofstream dump(DBG_DUMP_FILE, std::ios::app);
	    dump << f.x  << " " << f.y  << " " << f.z  << " "
	         << m.x  << " " << m.y  << " " << m.z  << " "
	         << k(0) << " " << k(1) << " " << k(2) << "\n";
	}
	catch (const std::exception& err)
	{
	    ROS_ERROR_STREAM(err.what());
	}
    }
#endif /* __MY_DEBUG__ > 1 */
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
    const Eigen::Matrix<double, 4, 1> result_f = _At_A.inverse() * _At_b;
    _f0 << result_f(0), result_f(1), result_f(2);
    _m  = -result_f(3)/G;

    const Eigen::Matrix<double, 6, 1> result_t = _Ct_C.inverse() * _Ct_d;
    _m0 << result_t(0), result_t(1), result_t(2);
    double mg = _m * G;
    _r0 << result_t(3)/mg, result_t(4)/mg, result_t(5)/mg;

#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("_At_A\n" << _At_A);
    ROS_INFO_STREAM("_At_A(inverse)\n" << _At_A.inverse());
    ROS_INFO_STREAM("_At_b\n" << _At_b);
    ROS_INFO_STREAM("result(force)\n" << result_f);
    // ROS_INFO_STREAM("_Ct_C\n" << _Ct_C);
    // ROS_INFO_STREAM("_Ct_C(inverse)\n" << _Ct_C.inverse());
    // ROS_INFO_STREAM("_Ct_d\n" << _Ct_d);
    // ROS_INFO_STREAM("result(torque)\n" << result_t);
#endif /* __MY_DEBUG__ */

    _At_A = Eigen::Matrix<double, 4, 4>::Zero();
    _At_b = Eigen::Matrix<double, 4, 1>::Zero();
    _Ct_C = Eigen::Matrix<double, 6, 6>::Zero();
    _Ct_d = Eigen::Matrix<double, 6, 1>::Zero();

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
		<< _f0(0) << _f0(1) << _f0(2)
		<< YAML::EndSeq;
	emitter << YAML::Key << KEY_M_OFFSET      << YAML::Value
		<< YAML::BeginSeq
		<< _m0(0) << _m0(1) << _m0(2)
		<< YAML::EndSeq;
	emitter << YAML::Key << KEY_R_OFFSET      << YAML::Value
		<< YAML::BeginSeq
		<< _r0(0) << _r0(1) << _r0(2)
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
