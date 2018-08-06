#include "ros/ros.h"
#include "o2as_fastener_gripper/dynamixel_controller.h"

using namespace dynamixel_controller;

// サービスサーバ
//コマンドの要請を受け取り、実行し結果を返す関数
bool kowa::dynamixelCommandMsgCallback(o2as_fastener_gripper::DynamixelWriteCommand::Request &req, o2as_fastener_gripper::DynamixelWriteCommand::Response &res)
{
	//check a control table of a Dynamixel
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

//コンストラクタ
kowa::kowa(void)
{
	int value = 10;                    // IDリスト要素数Max値
	uint8_t id_cnt = 0;                // ID件数
	id_list = new uint8_t[value];      // IDリスト

	dynamixel_driver = new DynamixelDriver;  // Dynamixel Driver初期化
	
	dynamixel_driver->init("/dev/ttyUSB0", 1000000);   // Dynamixelのデバイス名、baud_rateを指定して初期化

	printf("ok\n");
	//Dynamixel検索
	//第1引数：検索して見つかったDynamixelのIDを格納する配列
	//第2引数：見つかったDynamixelの数
	//第3引数：検索するDynamixel_IDの最大値
	if (dynamixel_driver->scan(id_list, &id_cnt, value))
	{
		//見つかれば情報出力
		for(int i=0; i < id_cnt; i++)
		{
			printf("[ID] %u, [Model Name] %s, [VERSION] %.1f\n",
							 id_list[i], dynamixel_driver->getModelName(id_list[i]), dynamixel_driver->getProtocolVersion());
		}
	}

	//サービスサーバ宣言
	//サービス名：dynamixel_command
	//サービス要請があった場合、kowa::dynamixelCommandMsgCallback関数を実行する
	//ros::ServiceServer dynamixel_command_server;
	dynamixel_command_server_ = node_handle_.advertiseService("dynamixel_write_command", &kowa::dynamixelCommandMsgCallback,this);
	dynamixel_info_server_ = node_handle_.advertiseService("dynamixel_read_state", &kowa::dynamixelReadMsgCallback,this);
}

//デストラクタ
kowa::~kowa(void){}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamixel_controller");    // ノード名：single_dynamixel_controller

	kowa test;  //クラス初期化

	ros::Rate loop_rate(1000);

	ROS_INFO("dynamixel_controller START");

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}