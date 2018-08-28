#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <vector>
#include <string>
#include <sstream>

#include <ros/ros.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_driver.h"

#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "o2as_fastener_gripper/DynamixelWriteCommand.h"
#include "o2as_fastener_gripper/DynamixelReadState.h"
#include "message_header.h"

namespace dynamixel_controller
{
class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  // ROS Service Server
  ros::ServiceServer dynamixel_info_server_;
  ros::ServiceServer dynamixel_command_server_;

  DynamixelDriver *dynamixel_driver[5];

  std::vector<std::string> access_point_list;

  std::vector<std::string> u2d2_connect_id_list;

  std::vector<uint32_t> baudrate_list =
  {
    9600,
    57600,
    115200,
    1000000
  };

  bool baudrate_flag = false;
  uint8_t *id_list;
  //uint32_t baundrate = 1000000;

 public:
  DynamixelController(void);
  ~DynamixelController(void);

  std::vector<std::string> StringSplit(const std::string &str, char sep)
  {
    std::vector<std::string> v;
    std::stringstream ss(str);
    std::string buffer;
    while( std::getline(ss, buffer, sep) ) {
        v.push_back(buffer);
    }
    return v;
  }

 private:
  bool initMotor(int32_t index, uint8_t motor_id, bool baudrate_flag);
  int32_t searchMotor(uint8_t motor_id);

  bool dynamixelCommandMsgCallback(o2as_fastener_gripper::DynamixelWriteCommand::Request &req,
                                   o2as_fastener_gripper::DynamixelWriteCommand::Response &res);
  
  bool dynamixelReadMsgCallback(o2as_fastener_gripper::DynamixelReadState::Request &req,
                                   o2as_fastener_gripper::DynamixelReadState::Response &res);
};
}

#endif //DYNAMIXEL_CONTROLLER_H
