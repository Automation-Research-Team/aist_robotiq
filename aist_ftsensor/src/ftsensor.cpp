/*!
 *  \file	ftsensor.cpp
 *  \brief	source file for a class for controlling FT300 force sensors
 */
#include "ftsensor.h"

namespace aist_ftsensor
{

/************************************************************************
*  class ftsensor								*
************************************************************************/
ftsensor::ftsensor(const std::string& name)
    :_nh(name),
     _publisher(_nh.advertise<wrench_t>("wrench", 100)),
     _subscriber(_nh.subscribe("/wrench_in", 100,
			       &ftsensor::wrench_callback, this)),
     _service(_nh.advertiseService("calibration",
				   &ftsensor::service_callback, this)),
     _listener(),
     _reference_frame("workspace_center"),
     _sensor_frame("ftsensor_wrench_link"),
     _rate(100),
     _mg(0, 0, 0),
     _f_offset(0, 0, 0),
     _m_offset(0, 0, 0),
     _r_offset(0, 0, 0),
     _get_sample(false),
     _Atranspose_A(Eigen::Matrix4f::Zero()),
     _Atranspose_b(Eigen::Vector4f::Zero()),
     _calibration_result(Eigen::Vector4f::Zero())
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
	_mg = tf::Vector3(0, 0, -9.8 * effector_mass);
    ROS_INFO_STREAM("mg=[" << _mg.x() << "," << _mg.y() << "," << _mg.z());

    std::vector<double> vec;
    if (_nh.getParam("f_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_f_offset.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("f_offset[" << _f_offset.x() << "," <<
		    _f_offset.y() << "," << _f_offset.z() << "]");

    vec.clear();
    if (_nh.getParam("m_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_m_offset.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("m_offset[" << _m_offset.x() << "," <<
		    _m_offset.y() << "," << _m_offset.z() << "]");

    vec.clear();
    if (_nh.getParam("r_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_r_offset.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("r_offset[" << _r_offset.x() << "," <<
		    _r_offset.y() << "," << _r_offset.z() << "]");

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
	}

	wrench_p	wrench(new wrench_t);
	wrench->header = wrench_msg->header;
	wrench->wrench = wrench_msg->wrench;
	wrench->header.frame_id = _sensor_frame;

	tf::Matrix3x3 Rt = T.getBasis();
	tf::Vector3   force  = Rt * _mg + _f_offset;
	tf::Vector3   torque = _r_offset.cross(Rt * _mg) + _m_offset;

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
ftsensor::take_sample(
		const tf::Matrix3x3& Rt, const geometry_msgs::Vector3& f)
{
#ifdef __MY_DEBUG__
    showMatrix3x3("Rt", Rt);
    showVector3("f", f);
#endif /* __MY_DEBUG__ */

    Eigen::Matrix4f Atranspose_A;
    Atranspose_A <<            1,            0,            0, -1*Rt[2].x(),
                               0,            1,            0, -1*Rt[2].y(),
                               0,            0,            1, -1*Rt[2].z(),
                    -1*Rt[2].x(), -1*Rt[2].y(), -1*Rt[2].z(),            1;
    _Atranspose_A += Atranspose_A;
#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("Atranspose_A\n" << Atranspose_A);
    ROS_INFO_STREAM("_Atranspose_A\n" << _Atranspose_A);
#endif /* __MY_DEBUG__ */

    Eigen::Vector4f Atranspose_b;
    Atranspose_b << f.x, f.y, f.z,
                   -1*Rt[2].x()*f.x+Rt[2].y()*f.y+Rt[2].z()*f.z;
    _Atranspose_b += Atranspose_b;
#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("Atranspose_b\n" <<  Atranspose_b);
    ROS_INFO_STREAM("_Atranspose_b\n" << _Atranspose_b);
#endif /* __MY_DEBUG__ */

    _get_sample = false;
}

void
ftsensor::compute_calibration()
{
    _calibration_result = _Atranspose_A.inverse() * _Atranspose_b;
#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("_Atranspose_A\n" << _Atranspose_A);
    ROS_INFO_STREAM("(_Atranspose_A)inverse\n" << _Atranspose_A.inverse());
    ROS_INFO_STREAM("_Atranspose_b\n" << _Atranspose_b);
    ROS_INFO_STREAM("_calibration_result\n" << _calibration_result);
#endif /* __MY_DEBUG__ */
}

void
ftsensor::save_calibration(const std::string& filepath)
{
    std::ofstream f(filepath);
    f << "f_offset:\n" <<
	"    " << _calibration_result(0) << "\n"
	"    " << _calibration_result(1) << "\n"
	"    " << _calibration_result(2) << "\n";
    f << "mg: " << _calibration_result(3) << "\n";
}

bool
ftsensor::service_callback(
		aist_ftsensor::Calibration::Request  &req,
		aist_ftsensor::Calibration::Response &res
		)
{
    _get_sample = false;
    res.Status = 9;

    ROS_INFO("#service_callback# request Mode=%d filePath=%s",
		req.Mode, req.FilePath.c_str());
    switch (req.Mode)
    {
    case 1: // take sample
        res.Status = 0;
	_get_sample = true;
	break;
    case 2: // compute calibration
	compute_calibration();
	res.Status = 0;
	break;
    case 3: // save calibration
	save_calibration(req.FilePath);
        res.Status = 0;
	break;
    case 4: // show samples
	ROS_INFO_STREAM("_Atranspose_A\n" << _Atranspose_A);
	ROS_INFO_STREAM("_Atranspose_b\n" << _Atranspose_b);
        res.Status = 0;
	break;
    default:
	break;
    }
    ROS_INFO("#service_callback# response.Status=%d", res.Status);
    return true;
}

double
ftsensor::rate() const
{
    return _rate;
}

}	// namespace aist_ftsensor
