#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, Grasp
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
import tf

z_ext = 0.80

def create_objects(scene, robot):
  scene.remove_world_object()

  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  
  """
  p.pose.position.x =  0.8
  p.pose.position.y =  0.0
  p.pose.position.z =  0.175 + z_ext
  scene.add_box("table1", p, (0.4, 0.6, 0.35))

  p.pose.position.x =  0.5
  p.pose.position.y =  0.6
  p.pose.position.z =  0.175 + z_ext
  scene.add_box("table2", p, (0.6, 0.4, 0.35))

  p.pose.position.x =  0.8
  p.pose.position.y =  1.2
  p.pose.position.z =  0.175 + z_ext
  scene.add_box("table3", p, (0.4, 0.6, 0.35))
  """

  p.pose.position.x =  0.8
  p.pose.position.y = -0.2
  p.pose.position.z =  0.5 + z_ext
  scene.add_box("part1", p, (0.03, 0.03, 0.3))

  p.pose.position.x =  0.8
  p.pose.position.y =  1.4
  p.pose.position.z =  0.5 + z_ext
  scene.add_box("part2", p, (0.03, 0.03, 0.3))

  rospy.sleep(1)

  print 'create objects'

def go_to_home(robot, robot_name):
  if robot_name is 'a_iiwa':
    robot.a_iiwa.set_named_target('home')
    robot.a_iiwa.go()
  elif robot_name is 'b_iiwa':
    robot.b_iiwa.set_named_target('home')
    robot.b_iiwa.go()
  elif robot_name is 'iiwa_two_robots':
    robot.iiwa_two_robots.set_named_target('home')
    robot.iiwa_two_robots.go()
  elif robot_name is 'iiwa':
    robot.iiwa.set_named_target('home')
    robot.iiwa.go()
  else:
    print robot_name, 'is unknown.'
  rospy.sleep(1)
  print robot_name, ': go to home'

def pose_targets(robot, robot_name, end_effector, poses):
  print robot_name, end_effector, ': pose_targets 1 (poses)', poses

  link_0 = robot.get_link(robot_name + '_link_0')
  link_0_pose = link_0.pose()

  for pose in poses:
    if pose.position.x > link_0_pose.pose.position.x:
      pose.position.x -= 0.1
    else:
      pose.position.x += 0.1
    pose.position.z += 0.2

  result = False

  print robot_name, end_effector, ': pose_targets 2 (poses)', poses
  if robot_name is 'a_iiwa':
    robot.a_iiwa.set_pose_targets(poses)
    result = robot.a_iiwa.go()
  elif robot_name is 'b_iiwa':
    robot.b_iiwa.set_pose_targets(poses)
    result = robot.b_iiwa.go()
  elif robot_name is 'iiwa':
    robot.iiwa.set_pose_targets(poses)
    result = robot.iiwa.go()
  else:
    print robot_name, 'is unknown.'

  print robot_name, ': end of pick (result)', result
  return result

def main(end_effector):
  scene = PlanningSceneInterface()
  robot = RobotCommander()
  rospy.sleep(1)

  go_to_home(robot, 'iiwa_two_robots')

  create_objects(scene, robot)

  objs = scene.get_objects()
  if 'part1' in objs:
    poses = objs['part1'].primitive_poses
    if pose_targets(robot, 'a_iiwa', end_effector, poses) in (
            MoveItErrorCodes.SUCCESS,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ):
      print 'continue'
    else:
      go_to_home(robot, 'a_iiwa')
      return
    go_to_home(robot, 'a_iiwa')

    rospy.sleep(1)

  objs = scene.get_objects()
  if 'part2' in objs:
    poses = objs['part2'].primitive_poses
    if pose_targets(robot, 'b_iiwa', end_effector, poses) in (
            MoveItErrorCodes.SUCCESS,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ):
      print 'continue'
    else:
      go_to_home(robot, 'b_iiwa')
      return
    go_to_home(robot, 'b_iiwa')

  return

if __name__ == '__main__':
  roscpp_initialize(sys.argv)
  rospy.init_node('moveit_trial', anonymous=True)

  end_effector = ''
  if len(sys.argv) > 1:
    end_effector = sys.argv[1]

  main(end_effector)
  print '----- end of main -----'

  rospy.spin()
  roscpp_shutdown()

