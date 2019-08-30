/*!
 *  \file	ft300.cpp
 *  \brief	source file for a class for controlling FT300 force sensors
 */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>		// for struct sockaddr_in
#include <arpa/inet.h>		// for inet_addr()
#include <netdb.h>		// for struct hostent, gethostbyname()
#include <errno.h>
#include "ftsensor.h"

namespace aist_ftsensor
{
/************************************************************************
*  static functions							*
************************************************************************/
static const char*
splitd(const char* s, double& val)
{
    for (; *s; ++s)
	if (*s == '+' || *s == '-' || *s == '.' || isdigit(*s))
	{
	    char*	end;
	    val = strtod(s, &end);
	    return end;
	}

    throw std::runtime_error("No strings representing numeric values found.");
    return nullptr;
}

/************************************************************************
*  class ftsensor							*
************************************************************************/
void
ftsensor::up_socket()
{
  // Check whether the socket is correctly opened.
    if (_socket < 0)
    {
	ROS_ERROR_STREAM("failed to open socket: " << strerror(errno));
	throw;
    }

  // Get hoastname and port from parameters.
    std::string	hostname;
    _nh.param<std::string>("hostname", hostname, "");
    int		port;
    _nh.param<int>("port", port, 63351);

  // Connect socket to hostname:port.
    auto	addr = inet_addr(hostname.c_str());
    if (addr == 0xffffffff)
    {
	const auto	h = gethostbyname(hostname.c_str());
	if (!h)
	{
	    ROS_ERROR_STREAM("unknown host name: " << hostname);
	    throw;
	}

	for (auto addr_ptr = (u_long**)h->h_addr_list; ; ++addr_ptr)
	{
	    if (!*addr_ptr)
		throw;
	    if (connect_socket(*(*addr_ptr), port))
		break;
	}
    }
    else
    {
	if (!connect_socket(addr, port))
	    throw;
    }
}

void
ftsensor::down_socket()
{
    if (_socket >= 0)
	close(_socket);
}

void
ftsensor::tick()
{
    std::array<char, 1024>	buf;
    const auto		nbytes = read(_socket, buf.data(), buf.size());
    if (nbytes < 0)
    {
	ROS_ERROR_STREAM("failed to read from socket.");
	throw;
    }
    buf[nbytes] = '\0';

    try
    {
	wrench_p	wrench(new wrench_t);
	wrench->header.frame_id = _sensor_frame;
	wrench->header.stamp    = ros::Time::now();

	const char*	s = buf.data();
	s = splitd(s, wrench->wrench.force.x);
	s = splitd(s, wrench->wrench.force.y);
	s = splitd(s, wrench->wrench.force.z);
	s = splitd(s, wrench->wrench.torque.x);
	s = splitd(s, wrench->wrench.torque.y);
	s = splitd(s, wrench->wrench.torque.z);

	transform_t     T;
	_listener.waitForTransform(_sensor_frame, _reference_frame,
				   wrench->header.stamp, ros::Duration(1.0));
	_listener.lookupTransform(_sensor_frame, _reference_frame,
				  wrench->header.stamp, T);
	const auto	colz = T.getBasis().getColumn(2);
	Eigen::Matrix<double, 3, 1> k;
	k << colz.x(), colz.y(), colz.z();

	if (_get_sample)
	{
	    take_sample(k, wrench->wrench.force, wrench->wrench.torque);
	    _get_sample = false;
	}

	Eigen::Matrix<double, 3, 1> force  = -_m*G*k + _f0;
	Eigen::Matrix<double, 3, 1> torque = -_r0.cross(_m*G*k) + _m0;

	wrench->wrench.force.x  -= force(0);
	wrench->wrench.force.y  -= force(1);
	wrench->wrench.force.z  -= force(2);
	wrench->wrench.torque.x -= torque(0);
	wrench->wrench.torque.y -= torque(1);
	wrench->wrench.torque.z -= torque(2);

	_publisher.publish(wrench);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }
}

bool
ftsensor::connect_socket(u_long s_addr, int port)
{
    sockaddr_in	server;
    server.sin_family	   = AF_INET;
    server.sin_port	   = htons(port);
    server.sin_addr.s_addr = s_addr;
    ROS_INFO_STREAM("trying to connect socket to "
		    << inet_ntoa(server.sin_addr) << ':' << port << "...");
    if (::connect(_socket, (sockaddr*)&server, sizeof(server)) == 0)
    {
	ROS_INFO_STREAM("succeeded.");
	return true;
    }
    else
    {
	ROS_ERROR_STREAM("failed: " << strerror(errno));
	return false;
    }
}

}	// namespace aist_ftsensor
