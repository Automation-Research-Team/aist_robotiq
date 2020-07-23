#ifndef TRIAL_ROBOTIQ_JOINT_HANDLE_H
#define TRIAL_ROBOTIQ_JOINT_HANDLE_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <control_toolbox/pid.h>
#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/controller_manager.h>

namespace aist_robotiq2f85_controller
{

class JointHandle : public robot_controllers::JointHandle
{
  enum CommandState
  {
    MODE_DISABLED,
    MODE_CONTROL_EFFORT,
    MODE_CONTROL_VELOCITY,
    MODE_CONTROL_POSITION
  };

public:
  JointHandle(const std::string name,
              const double position_lower_limit = 0.0,
              const double position_upper_limit = 0.0,
              const double velocity_limit = 0.0,
              const double effort_limit = 0.0,
              const bool continuous = true) :
    name_(name),
    actual_position_(0.0),
    actual_velocity_(0.0),
    actual_effort_(0.0)
  {
    ros::NodeHandle nh("~");

    // Load controller parameters
    position_pid_.init(ros::NodeHandle(nh, getName() + "/position"));
    velocity_pid_.init(ros::NodeHandle(nh, getName() + "/velocity"));

    // Set limits and continuous state
    position_limit_[0] = position_lower_limit;
    position_limit_[1] = position_upper_limit;
    velocity_limit_ = velocity_limit;
    effort_limit_ = effort_limit;
    continuous_ = continuous;

    reset();
  }
  virtual ~JointHandle()
  {
  }

  /**
   * @brief Set a position command for this joint.
   * @param position Position command in radians or meters.
   * @param velocity Velocity command in rad/sec or meters/sec.
   * @param effort Effort command in Nm or N.
   */
  virtual void setPosition(double position, double velocity, double effort)
  {
    ROS_DEBUG_STREAM_NAMED(log_named, log_named
                << "::setPosition(" << name_ << ") "
                << position << ", " << velocity << ", " << effort);
    // ControllerManager clears these all each cycle, so just
    // set mode and accumulate outputs of each controller.
    desired_position_ = position;
    desired_velocity_ = velocity;
    desired_effort_  = effort;
    mode_ = MODE_CONTROL_POSITION;
  }

  /**
   * @brief Set a velocity command for this joint.
   * @param velocity Velocity command in rad/sec or meters/sec.
   * @param effort Effort command in Nm or N.
   */
  virtual void setVelocity(double velocity, double effort)
  {
    desired_velocity_ = velocity;
    desired_effort_ = effort;
    if (mode_ != MODE_CONTROL_POSITION)
      mode_ = MODE_CONTROL_VELOCITY;
  }

  /**
   * @brief Set an effort command for this joint.
   * @param effort Effort command in Nm or N.
   */
  virtual void setEffort(double effort)
  {
    desired_effort_ = effort;
    if (mode_ != MODE_CONTROL_POSITION &&
        mode_ != MODE_CONTROL_VELOCITY)
      mode_ = MODE_CONTROL_EFFORT;
  }

  /** @brief Get the position of the joint in radians or meters. */
  virtual double getPosition()
  {
    ROS_DEBUG_STREAM_NAMED(log_named, log_named
                << "::getPosition(" << name_ << ") " << actual_position_);
    if (continuous_)
    {
      return angles::normalize_angle(actual_position_);
    }
    return actual_position_;
  }

  /** @brief Get the velocity of the joint in rad/sec or meters/sec. */
  virtual double getVelocity()
  {
    return actual_velocity_;
  }

  /** @brief Get applied effort of a joint in Nm or N. */
  virtual double getEffort()
  {
    return actual_effort_;
  }

  /* Is this joint continuous (has no position limits). */
  virtual bool isContinuous()
  {
    return continuous_;
  }

  /** @brief Get the minimum valid position command. */
  virtual double getPositionMin()
  {
    return position_limit_[0];
  }

  /** @brief Get the maximum valid position command. */
  virtual double getPositionMax()
  {
    return position_limit_[1];
  }

  /** @brief Get the maximum velocity command. */
  virtual double getVelocityMax()
  {
    return velocity_limit_;
  }

  /** @brief Get the maximum effort command. */
  virtual double getEffortMax()
  {
    return effort_limit_;
  }

  /** @brief Get the name of this joint. */
  virtual std::string getName()
  {
    return name_;
  } 

  /** @brief Reset the command. */
  virtual void reset()
  {
    ROS_DEBUG_STREAM_NAMED(log_named, log_named
                << "::reset(" << name_ << ") position="
                << desired_position_ << " velocity=" << desired_velocity_
                << " effort " << desired_effort_ << ")");
    desired_position_ = 0.0;
    desired_velocity_ = 0.0;
    desired_effort_ = 0.0;
    mode_ = MODE_DISABLED;
  }

  /** @brief Actually apply updates */
  void update(const ros::Time now, const ros::Duration dt)
  {
    ROS_DEBUG_STREAM_NAMED(log_named, log_named
        << "::update(" << name_ << ") position=("
        << desired_position_ << ", " << actual_position_ << ") velocity=("
        << desired_velocity_ << ", " << actual_velocity_ << ") effort=("
        << desired_effort_ << ", " << actual_effort_ << ")");

    actual_position_ = desired_position_;
    actual_velocity_ = desired_velocity_;
    actual_effort_ = desired_effort_;
  }

private:

  std::string name_;

  double actual_position_;
  double actual_velocity_;
  double actual_effort_;

  double desired_position_;
  double desired_velocity_;
  double desired_effort_;

  int mode_;
  
  control_toolbox::Pid position_pid_;
  control_toolbox::Pid velocity_pid_;

  double position_limit_[2];
  double velocity_limit_;
  double effort_limit_;

  bool continuous_;

  const std::string log_named = "JointHandle";

  // You no copy...
  JointHandle(const JointHandle&);
  JointHandle& operator=(const JointHandle&);
};

typedef boost::shared_ptr<JointHandle> JointHandlePtr;

}  // namespace aist_robotiq2f85_controller

#endif  // TRIAL_ROBOTIQ_JOINT_HANDLE_H
