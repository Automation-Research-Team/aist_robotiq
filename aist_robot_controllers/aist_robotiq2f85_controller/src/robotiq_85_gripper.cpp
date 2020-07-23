#include <pluginlib/class_list_macros.h>
#include <aist_robotiq2f85_controller/robotiq_85_gripper.h>

#include <fstream>

PLUGINLIB_EXPORT_CLASS(aist_robotiq2f85_controller::Robotiq85GripperController, robot_controllers::Controller)

namespace aist_robotiq2f85_controller
{

int Robotiq85GripperController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  ROS_INFO_STREAM_NAMED(log_named, log_named
                << "::init namespace=" << nh.getNamespace());

  Controller::init(nh, manager);
  manager_ = manager;

  // Setup Joints */
  std::string l_name, r_name;
  nh.param<std::string>(
                "l_gripper_joint", l_name, "robotiq_85_left_knuckle_joint");
  nh.param<std::string>(
                "r_gripper_joint", r_name, "robotiq_85_right_knuckle_joint");
  ROS_DEBUG_STREAM_NAMED(log_named, log_named
                << "::init name=(" << l_name << ", " << r_name << ")");

  left_  = JointHandlePtr(
                (JointHandle *)(manager_->getJointHandle(l_name).get()));
  right_ = JointHandlePtr(
                (JointHandle *)(manager_->getJointHandle(r_name).get()));

  // Checking to see if Joint Handles exists
  if (!left_)
  {
    ROS_ERROR_STREAM_NAMED(log_named, log_named
                << " Unable to retreive joint (" << l_name
                << "), Namespace: " << nh.getNamespace() << "/l_gripper_joint");
    return -1;  
  }
  
  if (!right_)
  {
    ROS_ERROR_STREAM_NAMED(log_named, log_named
                << " Unable to retreive joint (" << r_name
                << "), Namespace: " << nh.getNamespace() << "/r_gripper_joint");
    return -1;
  } 

  // Start action server
  server_.reset(new server_t(nh, "",
                boost::bind(&Robotiq85GripperController::executeCb, this, _1),
                false));
  server_->start();

  // Get Params
  nh.param<double>("min_gap_counts", min_gap_counts, 224.0);
  nh.param<double>("min_gap", position_limit_[0], 0.0);
  nh.param<double>("max_gap", position_limit_[1], 0.085);
  nh.param<double>("min_speed", velocity_limit_[0], 0.013);
  nh.param<double>("max_speed", velocity_limit_[1], 0.1);
  nh.param<double>("min_force", effort_limit_[0],  40.0);
  nh.param<double>("max_force", effort_limit_[1], 100.0);
  nh.param<double>("publish_rate", publish_rate, 60.0);
  ROS_DEBUG_STREAM_NAMED(log_named, log_named
        << "::init position=("
        << position_limit_[0] << ", " << position_limit_[1] << ") velocity=("
        << velocity_limit_[0] << ", " << velocity_limit_[1] << ") effort=("
        << effort_limit_[0] << ", " << effort_limit_[1] << ") rate="
        << publish_rate);

  nh.param<std::string>("ur_script_filepath", ur_script_filepath_,
      ros::package::getPath("robotiq_control")+"/src/robotiq_urscript.script");
  nh.param<double>("moving_duration/short", moving_duration_[0], 1.0);
  nh.param<double>("moving_duration/long",  moving_duration_[1], 4.0);
  ROS_INFO_STREAM_NAMED(log_named, log_named
        << "::init ur_script_filepath=" << ur_script_filepath_
        << "moving_duration=(" << moving_duration_[0]
        << ", " << moving_duration_[1] << ")");

  // Subscribe status from cmodel_urscript_driver
  std::string ur_state_topic;
  nh.param<std::string>("ur_state_topic", ur_state_topic,
                "/ur_driver/robot_mode_state");
  ur_state_sub_ = nh.subscribe<ur_msgs::RobotModeDataMsg>(ur_state_topic, 1,
                boost::bind(&Robotiq85GripperController::statusCb, this, _1));
  command_received_time_ = ros::Time(0);

  // Publish command to cmodel_urscript_driver
  std::string ur_script_topic;
  nh.param<std::string>("ur_script_topic", ur_script_topic,
                "/ur_driver/URScript");
  ur_script_pub_ = nh.advertise<std_msgs::String>(ur_script_topic, 1);

  ros::Duration(2.0).sleep();
  activate();

  // Set default to max
  goal_ = position_limit_[1];
  effort_ = effort_limit_[1];
  velocity_ = velocity_limit_[1];

  is_moving_  = false;
  is_closing_ = false;
  long_move_  = false;

  ur_script_ = "";

  initialized_ = true;

  return 0;
}

bool Robotiq85GripperController::start()
{
  ROS_INFO_STREAM_NAMED(log_named, log_named << "::start");

  if (!initialized_)
  {
    ROS_ERROR_STREAM_NAMED(log_named, log_named
                << " Unable to start, not initialized.");
    return false;
  }

  if (!server_->isActive())
  {
    ROS_ERROR_STREAM_NAMED(log_named, log_named
                << " Unable to start, action server is not active.");
    return false;
  }

  return true;
}

bool Robotiq85GripperController::stop(bool force)
{
  ROS_INFO_STREAM_NAMED(log_named, log_named << "::stop");

  if (!initialized_)
    return true;

  if (server_->isActive())
  {
    if (force)
    {
      // Shut down the action
      server_->setPreempted();
      return true;
    }

    // Do not abort unless forced
    return false;
  }

  // Just holding position, go ahead and stop us
  return true;
}

bool Robotiq85GripperController::reset()
{
  ROS_INFO_STREAM_NAMED(log_named, log_named << "::reset");
#if 0
  robotiq_msgs::CModelCommand command;
  command.rACT = 0;
  command.rGTO = 0;
  command.rATR = 0;
  command.rPR  = 0;
  command.rSP  = 0;
  command.rFR  = 0;
  return sendCommand(command);
#else
  return true;
#endif
}

bool Robotiq85GripperController::ready()
{
  return ( status_.gSTA == 3 && status_.gACT == 1 );
}

bool Robotiq85GripperController::stalled()
{
  return ( status_.gOBJ == 1 || status_.gOBJ == 2 );
}

bool Robotiq85GripperController::reached_goal(
                double cur_pos, double goal_pos, double tolerance)
{
  return ( status_.gPO == status_.gPR && fabs(cur_pos-goal_pos) < tolerance );
}

bool Robotiq85GripperController::activate()
{
  ROS_INFO_STREAM_NAMED(log_named, log_named << "::activate");

  robotiq_msgs::CModelCommand command;
  command.rACT = 1;
  command.rGTO = 1;
  command.rATR = 0;
  command.rPR  = 0;
  command.rSP  = 255;
  command.rFR  = 150;

  while (! ready())
  {
    if (! ros::ok())
    {
        server_->setPreempted();
        ROS_INFO_STREAM_NAMED(log_named, log_named
                << "::activate setPreempted");
        return false;
    }
    sendCommand(command);
  }
  return true;
}

std_msgs::String Robotiq85GripperController::buildCommand(
                robotiq_msgs::CModelCommand command)
{
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::buildCommand");

  if (ur_script_.size() <= 0)
  {
    try
    {
      ROS_INFO_STREAM_NAMED(log_named, log_named
                << "::buildCommand filePath=" << ur_script_filepath_);
      std::ifstream fs(ur_script_filepath_, std::ios::binary);
      char buf[1024+1];
      while (! fs.eof())
      {
        fs.read(buf, 1024);
        buf[fs.gcount()] = '\0';
        std::string str(buf);
        ur_script_ += str;
      }
      fs.close();
      ROS_DEBUG_STREAM_NAMED(log_named, log_named
                << "::buildCommand script=" << ur_script_);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM_NAMED(log_named, log_named
                << " can not open file(" << ur_script_filepath_ << ")");
    }
  }

  std_msgs::String msg;
  msg.data  = ur_script_;
  msg.data += ("rq_set_force(" + std::to_string(command.rFR) + ")\n");
  msg.data += ("rq_set_speed(" + std::to_string(command.rSP) + ")\n");
  msg.data += ("rq_move("      + std::to_string(command.rPR) + ")\n");
  msg.data += "end";
  ROS_DEBUG_STREAM_NAMED(log_named, log_named
                << "::buildCommand msg.data=" << msg.data);

  return msg;
}

bool Robotiq85GripperController::sendCommand(
                robotiq_msgs::CModelCommand command)
{
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand");


  if (command.rPR  == last_command_.rPR && command.rACT == last_command_.rACT &&
      command.rFR  == last_command_.rFR && command.rSP  == last_command_.rSP)
  {
    ROS_DEBUG_STREAM_NAMED(log_named, log_named
        << "::sendCommand It is same command, so it does not send.");
    return true;
  }

  ur_script_pub_.publish(buildCommand(command));
  last_command_ = command;

  status_.gACT = command.rACT;
  status_.gGTO = command.rGTO;
  status_.gSTA = 3;
  if (command.rGTO == 1 && command.rPR < 15)    // Gripper opens
  {
    status_.gOBJ = 0;   // Pretend that no object was grasped
    is_moving_  = true;
    is_closing_ = false;
  }
  if (command.rGTO == 1 && command.rPR > 200)
  {
    is_moving_  = true;
    is_closing_ = true;
  }
  command_received_time_ = ros::Time::now();

  status_.gFLT = 0;  // Fault
  status_.gPR  = command.rPR;
  status_.gCU  = 10; // Current

  if (command.rGTO == 1 && command.rSP < 50)    // Gripper opens
    long_move_ = true;

  ROS_INFO_NAMED(log_named, "%s::sendCommand finished rFR=%d rSP=%d rPR=%d",
                log_named.c_str(), command.rFR, command.rSP, command.rPR);

  return true;
}

bool Robotiq85GripperController::sendCommand(
                double position, double velocity, double effort)
{
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand"
        << " position=(" << position << ", "
        << position_limit_[0] << ", " << position_limit_[1]
        << ") velocity=(" << velocity << ", "
        << velocity_limit_[0] << ", " << velocity_limit_[1]
        << ") effort=(" << effort << ", "
        << effort_limit_[0] << "," << effort_limit_[1] << ")");

  robotiq_msgs::CModelCommand command;
  command.rACT = 1;
  command.rGTO = 1;
  command.rATR = 0;
  
  // position (0-255)
  double val = 0.0;
  val = (-min_gap_counts)/(position_limit_[1]-position_limit_[0])*(position-position_limit_[0])+min_gap_counts;
  command.rPR = (val < 0.0 ? 0: (val > min_gap_counts ? min_gap_counts: val));
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand pos=" << val);

  // speed (0-255)
  val = 255.0/(velocity_limit_[1]-velocity_limit_[0])*(velocity-velocity_limit_[0]);
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand vel=" << val);
  command.rSP = (val < 0.0 ? 0: (val > 255.0 ? 255: val));

  // force (0-255)
  val = 255.0/(effort_limit_[1]-effort_limit_[0])*(effort-effort_limit_[0]);
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand ef=" << val);
  command.rFR = (val < 0.0 ? 0: (val > 255.0 ? 255: val));

  return sendCommand(command);
}

void Robotiq85GripperController::update(const ros::Time& now, const ros::Duration& dt)
{
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::update");

  if (!initialized_)
    return;

  left_->setPosition(goal_, velocity_, effort_);
  right_->setPosition(0, 0, 0);

  if (! ready())
    activate();

  sendCommand(goal_, velocity_, effort_);
}

void Robotiq85GripperController::executeCb(const control_msgs::GripperCommandGoalConstPtr& goal)
{
  ROS_INFO_STREAM_NAMED(log_named, log_named
                << "::executeCb name=" << getName());

  control_msgs::GripperCommandFeedback feedback;
  control_msgs::GripperCommandResult result;

  status_.gOBJ = 0;

  if (!initialized_)
  {
    server_->setAborted(result, "Controller is not initialized.");
    return;
  }

  if (manager_->requestStart(getName()) != 0)
  {
    server_->setAborted(result, "Cannot execute, unable to start controller.");
    ROS_ERROR_STREAM_NAMED(log_named, log_named
                << " Cannot execute, unable to start controller.");
    return;
  }

  ROS_INFO_STREAM_NAMED(log_named, log_named
                << "::executeCb goal position=" << goal->command.position
                << " effort=" << goal->command.max_effort);

  if (goal->command.max_effort > effort_limit_[1])
    effort_ = effort_limit_[1];
  else if (goal->command.max_effort < effort_limit_[0])
    effort_ = effort_limit_[0];
  else
    effort_ = goal->command.max_effort;

  if (goal->command.position > position_limit_[1])
    goal_ = position_limit_[1];
  else if (goal->command.position < position_limit_[0])
    goal_ = position_limit_[0];
  else
    goal_ = goal->command.position;

  // Track position/time for stall detection
  float last_position = left_->getPosition() + right_->getPosition();
  ros::Time last_position_time = ros::Time::now();

  ros::Rate r(publish_rate);
  while (true)
  {
    // Abort detection
    if (server_->isPreemptRequested() || !ros::ok())
    {
      ROS_INFO_STREAM_NAMED(log_named, log_named << " Command preempted.");
      server_->setPreempted();
      break;
    }

    // Publish feedback before possibly completing
    feedback.position = left_->getPosition() + right_->getPosition();
    feedback.effort = left_->getEffort() + right_->getEffort();
    feedback.reached_goal = reached_goal(
                feedback.position, goal->command.position);
    feedback.stalled = stalled();
    server_->publishFeedback(feedback);

    ROS_DEBUG_STREAM_NAMED(log_named, log_named
                << "::executeCb position feedback=" << feedback.position
                << ", goal=" <<  goal->command.position
                << ", last=" << last_position << ")");

    // Goal detection
    if (feedback.reached_goal)
    {
      result.position = feedback.position;
      result.effort = feedback.effort;
      result.reached_goal = true;
      result.stalled = feedback.stalled;
      ROS_INFO_STREAM_NAMED(log_named, log_named << " Command Succeeded.");
      server_->setSucceeded(result);
      return;
    }

    // Stall detection
    if (fabs(feedback.position - last_position) > 0.005)
    {
      last_position = feedback.position;
      last_position_time = ros::Time::now();
    }
    else
    {
      if (ros::Time::now() - last_position_time > ros::Duration(
                                moving_duration_[(long_move_ ? 1: 0)]+1.0))
      {
        result.position = feedback.position;
        result.effort = feedback.effort;
        result.reached_goal = feedback.reached_goal;
        result.stalled = feedback.stalled;
        ROS_INFO_STREAM_NAMED(log_named, log_named
                                << " Gripper stalled, but succeeding.");
        server_->setSucceeded(result);
        return;
      }
    }

    r.sleep();
  }
}

std::vector<std::string> Robotiq85GripperController::getCommandedNames()
{
  std::vector<std::string> names;
  names.push_back(left_->getName());
  names.push_back(right_->getName());
  return names;
}

std::vector<std::string> Robotiq85GripperController::getClaimedNames()
{
  // Claimed == commanded.
  return getCommandedNames();
}

void Robotiq85GripperController::statusCb(
                const ur_msgs::RobotModeDataMsg::ConstPtr& msg)
{
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::statusCb msg=" << (*msg));

  if (! is_moving_)
    return;

  ros::Duration st = ros::Time::now() - command_received_time_;
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::statusCb moving " << st);
  if (st <= ros::Duration(moving_duration_[(long_move_ ? 1: 0)]))
    return;

  if (is_closing_)
    status_.gOBJ = 1;

  is_moving_  = false;
  is_closing_ = false;
  long_move_  = false;
  status_.gPO = status_.gPR;
  ROS_INFO_NAMED(log_named, "%s::statusCb update status gPO=%d",
            log_named.c_str(), status_.gPO);

  ros::Time update_time = ros::Time::now();
  left_->update(update_time, ros::Duration(0.1));
  right_->update(update_time, ros::Duration(0.1));
}

}  // namespace aist_robotiq2f85_controller
