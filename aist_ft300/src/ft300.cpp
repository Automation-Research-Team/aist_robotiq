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
#include "ft300.h"

namespace aist_ft300
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
*  class ft300								*
************************************************************************/
ft300::ft300(const std::string& name)
    :_nh(name),
     _socket(socket(AF_INET, SOCK_STREAM, 0)),
     _publisher(_nh.advertise<wrench_t>("ft300_wrench", 100)),
     _rate(100)
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

    _nh.param<std::string>("sensor_frame", _frame, "ft300_wrench_link");
    _nh.param<double>("rate", _rate, 100);
    ROS_INFO_STREAM("sensor_frame=" << _frame << ", rate=" << _rate);
    ROS_INFO_STREAM("aist_ft300 started.");
}

ft300::~ft300()
{
    if (_socket >= 0)
	close(_socket);
}

void
ft300::run()
{
    ros::Rate	rate(_rate);

    while (ros::ok())
    {
	tick();
	ros::spinOnce();
	rate.sleep();
    }
}

void
ft300::tick()
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
	wrench->header.frame_id = _frame;
	wrench->header.stamp    = ros::Time::now();

	const char*	s = buf.data();
	s = splitd(s, wrench->wrench.force.x);
	s = splitd(s, wrench->wrench.force.y);
	s = splitd(s, wrench->wrench.force.z);
	s = splitd(s, wrench->wrench.torque.x);
	s = splitd(s, wrench->wrench.torque.y);
	s = splitd(s, wrench->wrench.torque.z);

	_publisher.publish(wrench);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }
}

double
ft300::rate() const
{
    return _rate;
}

bool
ft300::connect_socket(u_long s_addr, int port)
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

}	// namespace aist_ft300
