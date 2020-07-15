#include <pluginlib/class_list_macros.h>
#include <aist_robotiq2f85_controller/robotiq_85_gripper.h>

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

  // Subscribe status from cmodel_urscript_driver
  std::string status_topic;
  nh.param<std::string>("status_topic", status_topic, "status");
  status_sub_ = nh.subscribe<robotiq_msgs::CModelStatus>(status_topic, 1,
                boost::bind(&Robotiq85GripperController::statusCb, this, _1));
  last_command_ = ros::Time(0);

  // Publish command to cmodel_urscript_driver
  std::string command_topic;
  nh.param<std::string>("command_topic", command_topic, "command");
  command_pub_ = nh.advertise<robotiq_msgs::CModelCommand>(command_topic, 10);

  // Set default to max
  goal_ = position_limit_[1];
  effort_ = effort_limit_[1];
  velocity_ = velocity_limit_[1];

  command_.rACT = 1;
  command_.rGTO = 1;
  command_.rATR = 0;
  command_.rPR  = 0;
  command_.rSP  = 255;
  command_.rFR  = 0;
  command_pub_.publish(command_);

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

#if 0
  if (ros::Time::now() - last_command_ > ros::Duration(3.0))
  {
    ROS_ERROR_STREAM_NAMED(log_named, log_named << "Unable to start, no goal.");
    return false;
  }
#endif

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
  command_.rACT = 0;
  command_.rGTO = 0;
  command_.rATR = 0;
  command_.rPR  = 0;
  command_.rSP  = 0;
  command_.rFR  = 0;
  command_pub_.publish(command_);

  return true;
}

void Robotiq85GripperController::sendCommand(
                double position, double velocity, double effort)
{
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand"
        << " position=(" << position << ", "
        << position_limit_[0] << ", " << position_limit_[1]
        << ") velocity=(" << velocity << ", "
        << velocity_limit_[0] << ", " << velocity_limit_[1]
        << ") effort=(" << effort << ", "
        << effort_limit_[0] << "," << effort_limit_[1] << ")");
  
  // position (0-255)
  double val = 0.0;
  val = (-min_gap_counts)/(position_limit_[1]-position_limit_[0])*(position-position_limit_[0])+min_gap_counts;
  command_.rPR = (val < 0.0 ? 0: (val > min_gap_counts ? min_gap_counts: val));
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand pos=" << val);

  // speed (0-255)
  val = 255.0/(velocity_limit_[1]-velocity_limit_[0])*(velocity-velocity_limit_[0]);
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand vel=" << val);
  command_.rSP = (val < 0.0 ? 0: (val > 255.0 ? 255: val));

  // force (0-255)
  val = 255.0/(effort_limit_[1]-effort_limit_[0])*(effort-effort_limit_[0]);
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::sendCommand ef=" << val);
  command_.rFR = (val < 0.0 ? 0: (val > 255.0 ? 255: val));

  command_pub_.publish(command_);
}


void Robotiq85GripperController::update(const ros::Time& now, const ros::Duration& dt)
{
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::update");

  if (!initialized_)
    return;

  left_->setPosition(goal_, velocity_, effort_);
  right_->setPosition(0, 0, 0);

  sendCommand(goal_, velocity_, effort_);
}

void Robotiq85GripperController::executeCb(const control_msgs::GripperCommandGoalConstPtr& goal)
{
  ROS_INFO_STREAM_NAMED(log_named, log_named
                << "::executeCb name=" << getName());

  control_msgs::GripperCommandFeedback feedback;
  control_msgs::GripperCommandResult result;

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

  ROS_DEBUG_STREAM_NAMED(log_named, log_named
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
    feedback.reached_goal = false;
    feedback.stalled = false;
    server_->publishFeedback(feedback);

    // Goal detection
    if (fabs(feedback.position - goal->command.position) < 0.002)
    {
      result.position = feedback.position;
      result.effort = feedback.effort;
      result.reached_goal = true;
      result.stalled = false;
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
      if (ros::Time::now() - last_position_time > ros::Duration(2.0))
      {
        result.position = feedback.position;
        result.effort = feedback.effort;
        result.reached_goal = false;
        result.stalled = true;
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
                const robotiq_msgs::CModelStatus::ConstPtr& msg)
{
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::statusCb");
  last_command_ = ros::Time::now();
  left_->update(last_command_, ros::Duration(2.0));
  right_->update(last_command_, ros::Duration(2.0));
}

}  // namespace aist_robotiq2f85_controller
