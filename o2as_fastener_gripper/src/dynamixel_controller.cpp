#include "ros/ros.h"
#include "o2as_fastener_gripper/dynamixel_controller.h"

using namespace dynamixel_controller;

bool kowa::dynamixelCommandMsgCallback(o2as_fastener_gripper::DynamixelWriteCommand::Request &req, o2as_fastener_gripper::DynamixelWriteCommand::Response &res)
{
	uint8_t motor_id = req.motor_id;
	std::string addr = req.addr_name;
	int64_t value = req.value;

	if (dynamixel_driver->writeRegister(motor_id, addr.c_str(), value))
	{
		ROS_DEBUG("OK\n");
		res.comm_result = true;
	}
	else
	{
		 ROS_DEBUG("NG\n");
		 res.comm_result = false;
	}
	return true;
}

bool kowa::dynamixelReadMsgCallback(o2as_fastener_gripper::DynamixelReadState::Request &req, o2as_fastener_gripper::DynamixelReadState::Response &res)
{
	int32_t item_status = 0;
	if(dynamixel_driver->readRegister(req.motor_id, req.item_name.c_str(), &item_status))
	{
		res.value = item_status;
		res.comm_result = true;
	}
	else
	{
		res.comm_result = false;
	}
	return true;
}

kowa::kowa(void)
{
	int value = 10;
	uint8_t id_cnt = 0;
	id_list = new uint8_t[value];
	std::string connect_name;
	int baundrate;

	ros::NodeHandle node_private("~");
	node_private.param<std::string>("connect_name",connect_name,"");
	node_private.param<int>("baundrate",baundrate,0);

	dynamixel_driver = new DynamixelDriver;
	dynamixel_driver->init(connect_name.c_str(), baundrate);

	if (dynamixel_driver->scan(id_list, &id_cnt, value))
	{
		for(int i=0; i < id_cnt; i++)
		{
			printf("[ID] %u, [Model Name] %s, [VERSION] %.1f\n",
							 id_list[i], dynamixel_driver->getModelName(id_list[i]), dynamixel_driver->getProtocolVersion());
		}
	}

	dynamixel_command_server_ = node_handle_.advertiseService("dynamixel_write_command", &kowa::dynamixelCommandMsgCallback,this);
	dynamixel_info_server_ = node_handle_.advertiseService("dynamixel_read_state", &kowa::dynamixelReadMsgCallback,this);
}

kowa::~kowa(void){}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamixel_controller");

	kowa test;

	ros::Rate loop_rate(1000);
	
	ROS_INFO("dynamixel_controller START");

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}