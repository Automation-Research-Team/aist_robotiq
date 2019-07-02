/*!
 *  \file	main.cpp
 *  \brief	wrench publisher for FT300 force sensors
 */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>		// for struct sockaddr_in
#include <arpa/inet.h>		// for inet_addr()
#include <netdb.h>		// for struct hostent, gethostbyname()
#include <errno.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

namespace ft300_wrench
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
*  class FT300								*
************************************************************************/
class FT300
{
  private:
    using wrench_t	= geometry_msgs::WrenchStamped;

  public:
		FT300()							;
		~FT300()						;

    void	run()							;

  private:
    bool	connect_socket(u_long hostname, int port)		;

  private:
    ros::NodeHandle		_nh;
    const int			_socket;
    const ros::Publisher	_publisher;
    double			_frequency;
    wrench_t			_wrench;
};

FT300::FT300()
    :_nh("~"),
     _socket(socket(AF_INET, SOCK_STREAM, 0)),
     _publisher(_nh.advertise<wrench_t>("ft300_wrench", 100)),
     _frequency(100),
     _wrench()
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

    _nh.param<double>("frequency", _frequency, 100);
    _nh.param<std::string>("sensor_frame", _wrench.header.frame_id,
			   "ft300_wrench_link");
    ROS_INFO_STREAM("frequency=" << _frequency
		    << ", sensor_frame=" << _wrench.header.frame_id);
    ROS_INFO_STREAM("aist_ft300_wrench started.");
}

FT300::~FT300()
{
    if (_socket >= 0)
	close(_socket);
}

void
FT300::run()
{
    ros::Rate	rate(_frequency);

    while (ros::ok())
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
	    _wrench.header.stamp = ros::Time::now();

	    const char*	s = buf.data();
	    s = splitd(s, _wrench.wrench.force.x);
	    s = splitd(s, _wrench.wrench.force.y);
	    s = splitd(s, _wrench.wrench.force.z);
	    s = splitd(s, _wrench.wrench.torque.x);
	    s = splitd(s, _wrench.wrench.torque.y);
	    s = splitd(s, _wrench.wrench.torque.z);

	    _publisher.publish(_wrench);
	}
	catch (const std::exception& err)
	{
	    ROS_ERROR_STREAM(err.what());
	}

	ros::spinOnce();
	rate.sleep();
    }
}

bool
FT300::connect_socket(u_long s_addr, int port)
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

}	// namespace ft300_wrench

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "ft300_wrench");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	ft300_wrench::FT300	node;
	node.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }
    catch (...)
    {
	std::cerr << "Unknown error." << std::endl;
	return 1;
    }

    return 0;
}
