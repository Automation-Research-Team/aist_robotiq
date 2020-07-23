#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_controllers_interface/controller_manager.h>
#include <aist_robotiq2f85_controller/joint_handle.h>

namespace aist_robotiq2f85_controller
{

class Robotiq85GripperControllerNode
{
public:
  Robotiq85GripperControllerNode(ros::NodeHandle nh);
  ~Robotiq85GripperControllerNode();

  virtual void Init();
  void OnUpdate(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_;
  robot_controllers::ControllerManager controller_manager_;

  ros::Timer timer_;

  std::vector<JointHandlePtr> joints_;
  ros::Time prevUpdateTime_;
  ros::Time last_publish_;
  ros::Publisher joint_state_pub_;

  const std::string log_named = "Robotiq85GripperControllerNode";

};

Robotiq85GripperControllerNode::Robotiq85GripperControllerNode(
                ros::NodeHandle nh) : nh_(nh)
{
}

Robotiq85GripperControllerNode::~Robotiq85GripperControllerNode()
{
}

void Robotiq85GripperControllerNode::Init()
{
  ROS_INFO_STREAM_NAMED(log_named, log_named
                << "::Init namespace=" << nh_.getNamespace());

  std::vector<std::string> names = {
        "robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint" };
  for (auto name : names)
  { 
    ROS_INFO_STREAM_NAMED(log_named, log_named << "::Init " << name);
    JointHandlePtr handle(new JointHandle(name));
    joints_.push_back(handle);
    robot_controllers::JointHandlePtr h(handle);
    controller_manager_.addJointHandle(h);
  }
  
  // Init controllers
  controller_manager_.init(nh_);
  
  // Publish joint states only after controllers are fully ready
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  timer_ = nh_.createTimer(ros::Duration(0.5),
              boost::bind(&Robotiq85GripperControllerNode::OnUpdate, this, _1));

  ROS_INFO_STREAM_NAMED(log_named, log_named
                << " Finished initializing Robotiq85GripperControllerNode");
}

void Robotiq85GripperControllerNode::OnUpdate(const ros::TimerEvent& event)
{
  if (!ros::ok())
    return;

  ros::Time now = ros::Time::now();
  double dt = now.toSec() - prevUpdateTime_.toSec();
  ROS_DEBUG_STREAM_NAMED(log_named, log_named << "::OnUpdate dt=" << dt);

  controller_manager_.update(now, ros::Duration(dt));

#if 0
  for (size_t i = 0; i < joints_.size(); ++i)
    joints_[i]->update(now, ros::Duration(dt));
#endif

  // Limit publish rate
  if (now - last_publish_ < ros::Duration(0.01))
    return;

  // Publish joint_state message
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time(now.toSec());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    js.name.push_back(joints_[i]->getName());
    js.position.push_back(joints_[i]->getPosition());
    js.velocity.push_back(joints_[i]->getVelocity());
    js.effort.push_back(joints_[i]->getEffort());
  }
  joint_state_pub_.publish(js);

  last_publish_ = now;
}

}   // namespace aist_robotiq2f85_controller

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Robotiq85GripperControllerNode");
  ros::NodeHandle nh("~");
  aist_robotiq2f85_controller::Robotiq85GripperControllerNode node(nh);

  node.Init();

  ros::spin();

  return 0;
}

