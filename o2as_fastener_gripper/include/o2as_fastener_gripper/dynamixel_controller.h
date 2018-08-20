/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <ros/ros.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_driver.h"

#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "o2as_fastener_gripper/DynamixelWriteCommand.h"
#include "o2as_fastener_gripper/DynamixelReadState.h"
#include "message_header.h"

namespace dynamixel_controller
{
class kowa
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_status_pub_;

  // ROS Topic Subscriber
  ros::Subscriber dynamixel_status_msg_sub_;

  // ROS Service Server
  ros::ServiceServer dynamixel_info_server_;
  ros::ServiceServer dynamixel_command_server_;

  DynamixelDriver *dynamixel_driver;

  std::string device_name_;
  uint32_t dxl_baud_rate_;
  uint8_t dxl_id_;
  uint8_t *id_list;
  uint8_t get_id;  

 public:
  kowa(void);
  ~kowa(void);
  bool controlLoop();

  void OKgoogle(const dynamixel_workbench_msgs::XL::ConstPtr &msg);

 private:
  void initSingleDynamixelMonitor(void);
  void shutdownSingleDynamixelMonitor(void);

  // TODO : Add new Dynamixel
  void initDynamixelStatePublisher(void);
  void initDynamixelInfoServer(void);
  void initDynamixelCommandServer(void);
  // TODO : Add new Dynamixel
  void dynamixelStatePublish(void);

  bool showDynamixelControlTable(void);
  bool checkValidationCommand(std::string cmd);
  bool changeId(uint8_t new_id);
  bool changeBaudrate(uint32_t new_baud_rate);
  bool changeProtocolVersion(float ver);

  bool dynamixelInfoMsgCallback(dynamixel_workbench_msgs::GetDynamixelInfo::Request &req,
                                dynamixel_workbench_msgs::GetDynamixelInfo::Response &res);

  bool dynamixelCommandMsgCallback(o2as_fastener_gripper::DynamixelWriteCommand::Request &req,
                                   o2as_fastener_gripper::DynamixelWriteCommand::Response &res);
  
  bool dynamixelReadMsgCallback(o2as_fastener_gripper::DynamixelReadState::Request &req,
                                   o2as_fastener_gripper::DynamixelReadState::Response &res);

  bool Moving_Speed(DynamixelDriver *dynamixel_driver,uint8_t dxl_id_,int32_t data);
};
}

#endif //DYNAMIXEL_CONTROLLER_H
