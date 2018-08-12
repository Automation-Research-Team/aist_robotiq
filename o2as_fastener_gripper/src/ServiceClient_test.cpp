#include "o2as_fastener_gripper/ServiceClient_test.h"

using namespace ServerClient_test;

bool shutdownSingleDynamixelController(void)
{
  ros::shutdown();
  return true;
}

int getch()
{
  struct termios oldt, newt;
  int ch;

  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 1;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

int kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

//サービス要請コマンド
bool sendCommandMsg(std::string cmd, std::string addr, int64_t value)
{
  ros::NodeHandle node_handle_;
  ros::ServiceClient dynamixel_command_client_;
  dynamixel_command_client_ = node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel_command");

  //メッセージタイプdynamixel_workbench_msgsで宣言
  dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

  //引数設定
  set_dynamixel_command.request.command   = cmd;
  set_dynamixel_command.request.addr_name = addr;
  set_dynamixel_command.request.value     = value;

  //サービス要請し、結果がTrueなら成功。Falseなら失敗。
  if (dynamixel_command_client_.call(set_dynamixel_command))
  {
    printf("サービス呼び出し成功\n");
    if (!set_dynamixel_command.response.comm_result)
      return false;
    else
      return true;
  }
  else
  {
    printf("サービス呼び出し失敗\n");
  }
}

//サービス要請コマンド
bool sendCommandMsg2(std::string cmd, std::string addr, int64_t value)
{
  ros::NodeHandle node_handle_;
  ros::ServiceClient dynamixel_command_client_;
  dynamixel_command_client_ = node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel_read");

  //メッセージタイプdynamixel_workbench_msgsで宣言
  dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

  //引数設定
  set_dynamixel_command.request.command   = cmd;
  set_dynamixel_command.request.addr_name = addr;
  set_dynamixel_command.request.value     = value;

  //サービス要請し、結果がTrueなら成功。Falseなら失敗。
  if (dynamixel_command_client_.call(set_dynamixel_command))
  {
    printf("サービス呼び出し成功\n");
    if (!set_dynamixel_command.response.comm_result)
      return false;
    else
      return true;
  }
  else
  {
    printf("サービス呼び出し失敗\n");
    return false;
  }
}


bool controlLoop()
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;
  bool valid_cmd = false;

  printf("[CMD]");

  //入力待ち
  fgets(input, sizeof(input), stdin);

  char *p;
  if ((p = strchr(input, '\n'))!= NULL) *p = '\0';
  fflush(stdin);

  if (strlen(input) == 0) return false;

  token = strtok(input, " ");

  if (token == 0) return false;

  strcpy(cmd, token);
  token = strtok(0, " ");
  num_param = 0;

  while (token != 0)
  {
  strcpy(param[num_param++], token);
  token = strtok(0, " ");
  }

  //コマンドの各処理
  if (strcmp(cmd, "info") == 0)
  {
  if (!sendCommandMsg2("info", param[0], atoi(param[1])))
      printf("It didn't works\n");
  else
      printf("OK!!\n");
  }
  else if (num_param == 1)
  {
  if (sendCommandMsg("addr", cmd, atoi(param[0])))
      printf("It works!!\n");
  else
      printf("It didn't works!!\n");
  }
  else
  {
  printf("Invalid command.\n");
  }
}

//メイン関数
int main(int argc, char **argv)
{
  //ノードの初期化
  //ノード名：single_dynamixel_controller
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle node_handle_;
  //ループ周期を設定
  ros::Rate loop_rate(250);

  printf("ServerClient_test START\n");

  //Ctrl+Cが入力されるまで・ros::shutdownが実行されるまでループ処理
  while (ros::ok())
  {
    controlLoop();

    ros::spinOnce();
    
    loop_rate.sleep();
  }

  return 0;
}
