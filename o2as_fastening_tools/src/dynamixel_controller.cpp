#include <fstream>
#include "ros/ros.h"
#include "o2as_fastening_tools/dynamixel_controller.h"

using namespace dynamixel_controller;

bool DynamixelController::dynamixelCommandMsgCallback(o2as_fastening_tools::DynamixelWriteCommand::Request &req, o2as_fastening_tools::DynamixelWriteCommand::Response &res)
{
	uint8_t motor_id = req.motor_id;
	std::string item = req.item_name;
	int64_t value = req.value;

	res.comm_result = true;

    int result_index = DynamixelController::searchMotor(motor_id);
	if (result_index == -1)
	{
		ROS_ERROR("The specified Motor_ID (%d) can not be found.",motor_id);
		res.comm_result = false;
	}
	else
	{
		if (dynamixel_driver[result_index]->writeRegister(motor_id, item.c_str(), value)) res.comm_result = true;
	    else res.comm_result = false;
	}
}

bool DynamixelController::dynamixelReadMsgCallback(o2as_fastening_tools::DynamixelReadState::Request &req, o2as_fastening_tools::DynamixelReadState::Response &res)
{
	uint8_t motor_id = req.motor_id;
	int32_t item_status = 0;

    int result_index = DynamixelController::searchMotor(motor_id);
	if (result_index == -1)
	{
		ROS_ERROR("The specified Motor_ID (%d) can not be found.",motor_id);
		res.comm_result = false;
	}
	else
	{
		if (dynamixel_driver[result_index]->readRegister(motor_id, req.item_name.c_str(), &item_status))
		{
			res.value = item_status;
			res.comm_result = true;
		}
	    else res.comm_result = false;
	}
}

int DynamixelController::searchMotor(uint8_t motor_id)
{
	std::vector<std::string> split_list;

	for(int index=0; index < u2d2_connect_id_list.size(); index++)
	{
		if (u2d2_connect_id_list[index] != "")
		{
			split_list = DynamixelController::StringSplit(u2d2_connect_id_list[index],',');
			for(int i=0; i<split_list.size(); i++) if (std::stoi(split_list[i]) == motor_id) return index;
		}
	}

	return -1;
}

bool DynamixelController::initMotor(int32_t index, uint8_t motor_id, bool baudrate_flag)
{
	if (baudrate_flag)
	{
		if (! dynamixel_driver[index]->writeRegister(motor_id, "Baud_Rate", 1000000))
	    {
	    	ROS_ERROR("The value of Baud_Rate could not be set to 1000000.");
	    	return false;
	    }
	}

	if (! dynamixel_driver[index]->writeRegister(motor_id, "Torque_Enable", 0))
	{
		ROS_ERROR("The value of Torque_Enable could not be set to 0.");
		return false;
	}
	if (! dynamixel_driver[index]->writeRegister(motor_id, "Control_Mode", 1))
	{
		ROS_ERROR("The value of Control_Mode could not be set to 1.");
		return false;
	}
	if (! dynamixel_driver[index]->writeRegister(motor_id, "Torque_Enable", 1))
	{
		ROS_ERROR("The value of Torque_Enable could not be set to 1.");
		return false;
	}
	// if (! dynamixel_driver[index]->writeRegister(motor_id, "Torque_Limit", 1023))
	// {
	// 	ROS_ERROR("The value of Torque_Limit could not be set to 1023.");
	// 	return false;
	// }

	return true;
}

DynamixelController::DynamixelController(void)
{
	int max_access = 0;
    int value = 10;
	bool exist_flag = true;
	std::string access;
	uint8_t id_cnt = 0;
	id_list = new uint8_t[value];
	
	// check access point
    ros::NodeHandle nh("~");
	if (! nh.getParam("max_access", max_access))
	{
		ROS_ERROR("No exist param. (max_access)");
		return;
	}

	for(int no=1; no <= max_access; no++){
		access = "serial_port_" + std::to_string(no);
		if (nh.getParam(access, access)) access_point_list.push_back(access);
		else
		{
			ROS_ERROR("No exist param. (%s)", access.c_str());
			exist_flag = false;
		}
	}
	if (! exist_flag) return;


    //init and serach XL-320
	for(int index=0; index < access_point_list.size(); index++)
	{
		u2d2_connect_id_list.push_back("");
		std::ifstream ifs(access_point_list[index]);
	    
		if (ifs.is_open())
	    {
			dynamixel_driver[index] = new DynamixelDriver;
	        for(int n=0; n<4; n++)
			{
				exist_flag = false;
				dynamixel_driver[index]->init(access_point_list[index].c_str(), baudrate_list[n]);
			    ROS_INFO("%d",baudrate_list[n]);
			    if (dynamixel_driver[index]->scan(id_list, &id_cnt, value))
			    {
			    	for(int i=0; i < id_cnt; i++)
			    	{
			    		if (i==0) ROS_INFO("Connection wsa found. (%s)" , access_point_list[index].c_str());
			    		ROS_INFO("[ID] %u, [Model Name] %s, [VERSION] %.1f",
			    						id_list[i], dynamixel_driver[index]->getModelName(id_list[i]), dynamixel_driver[index]->getProtocolVersion());
			    		
			    		//Initial Setting(XL320)
						if (baudrate_list[n] != 1000000) baudrate_flag = true;
						else baudrate_flag = false;

			    		if (DynamixelController::initMotor(index, id_list[i], baudrate_flag))
			    		{
			    			if (u2d2_connect_id_list[index] == "") u2d2_connect_id_list[index] = std::to_string(id_list[i]);
			    			else u2d2_connect_id_list[index] = u2d2_connect_id_list[index] + "," + std::to_string(id_list[i]);
			    		}
			    	}
					exist_flag = true;
					break;
			    }
			}
			if (! exist_flag)
			{
				ROS_INFO("Connection wsa found. (%s)" , access_point_list[index].c_str());
				ROS_ERROR("Can not find XL-320.");
			}
	    }
	    else
		{
			ROS_WARN("No exist File (%s)",access_point_list[index].c_str());
			access_point_list[index] = "";
		}
	}
	
	dynamixel_command_server_ = node_handle_.advertiseService("dynamixel_write_command", &DynamixelController::dynamixelCommandMsgCallback,this);
	dynamixel_info_server_ = node_handle_.advertiseService("dynamixel_read_state", &DynamixelController::dynamixelReadMsgCallback,this);
}

DynamixelController::~DynamixelController(void){}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamixel_controller");

	DynamixelController controller;

	ros::Rate loop_rate(1000);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
