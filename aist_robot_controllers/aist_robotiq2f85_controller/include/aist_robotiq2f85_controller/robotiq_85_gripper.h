#ifndef ROBOT_CONTROLLERS_ROBOTIQ_85_GRIPPER_H
#define ROBOT_CONTROLLERS_ROBOTIQ_85_GRIPPER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_manager.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>

#include <robotiq_msgs/CModelCommand.h>
#include <robotiq_msgs/CModelStatus.h>

#include <aist_robotiq2f85_controller/joint_handle.h>

namespace aist_robotiq2f85_controller
{

using namespace robot_controllers;

/**
 * @brief Controller for a parallel jaw gripper, is really only intended for
 *        use in simulation.
 */
class Robotiq85GripperController : public Controller
{
  typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> server_t;

public:
  Robotiq85GripperController() :
    initialized_(false) {}
  virtual ~Robotiq85GripperController() {}

  /**
   * @brief Initialize the controller and any required data structures.
   * @param nh Node handle for this controller.
   * @param manager The controller manager instance, this is needed for the
   *        controller to get information about joints, etc.
   * @returns 0 if succesfully configured, negative values are error codes.
   */
  virtual int init(ros::NodeHandle& nh, ControllerManager* manager);

  /**
   * @brief Attempt to start the controller. This should be called only by the
   *        ControllerManager instance.
   * @returns True if successfully started, false otherwise.
   */
  virtual bool start();

  /**
   * @brief Attempt to stop the controller. This should be called only by the
   *        ControllerManager instance.
   * @param force Should we force the controller to stop? Some controllers
   *        may wish to continue running until they absolutely have to stop.
   * @returns True if successfully stopped, false otherwise.
   */
  virtual bool stop(bool force);

  /**
   * @brief Cleanly reset the controller to it's initial state. Some controllers
   *        may choose to stop themselves. This is mainly used in the case of the
   *        the robot exiting some fault condition.
   * @returns True if successfully reset, false otherwise.
   */
  virtual bool reset();

  /**
   * @brief This is the update loop for the controller.
   * @param time The system time.
   * @param dt The timestep since last call to update.
   */
  virtual void update(const ros::Time& now, const ros::Duration& dt);

  /** @brief Get the type of this controller. */
  virtual std::string getType()
  {
    return "aist_robotiq2f85_controller/Robotiq85GripperController";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

private:
  void executeCb(const control_msgs::GripperCommandGoalConstPtr& goal);

  bool initialized_;
  ControllerManager* manager_;

  JointHandlePtr left_;
  JointHandlePtr right_;

  double goal_, effort_, velocity_;

  double position_limit_[2];
  double effort_limit_[2];
  double velocity_limit_[2];
  double min_gap_counts;

  double publish_rate;

  boost::shared_ptr<server_t> server_;

  /* for cmodel_urscript_driver */
  ros::Publisher command_pub_;
  ros::Subscriber status_sub_;
  ros::Time last_command_;
  robotiq_msgs::CModelCommand command_;

  const std::string log_named = "Robotiq85GripperController";

  void statusCb(const robotiq_msgs::CModelStatus::ConstPtr& msg);
  void sendCommand(double position, double velocity, double effort);

};

}  // namespace aist_robotiq2f85_controller

#endif  // ROBOT_CONTROLLERS_ROBOTIQ_85_GRIPPER_H
