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

def go_to_named_target(robot, name, pose_name):
  group = robot.get_group(name)
  group.set_named_target(pose_name)
  group.go()
  rospy.sleep(1)
  print name, ': go to', pose_name

def go_to_home(robot, name):
  go_to_named_target(robot, name, 'home')

def go_to_standing(robot, name):
  go_to_named_target(robot, name, 'standing')

def pose_targets(robot, name, end_effector, poses):
  print name, end_effector, ': pose_targets 1 (poses)', poses

  group = robot.get_group(name)

  link_0 = None
  if 'a_iiwa' in name:
    link_0 = robot.get_link('a_iiwa_link_0')
  elif 'b_iiwa' in name:
    link_0 = robot.get_link('b_iiwa_link_0')
  link_0_pose = link_0.pose()

  ee_link = group.get_end_effector_link()
  print name, end_effector, ': pose_targets (ee_link)', ee_link

  if 'robotiq_85_tip_link' in ee_link:
    q_tf = tf.transformations.quaternion_from_euler(0, 0, 0)
  else:
    q_tf = tf.transformations.quaternion_from_euler(0, 3.14/2, 0)

  for pose in poses:
    if pose.position.x > link_0_pose.pose.position.x:
      pose.position.x -= 0.1
    else:
      pose.position.x += 0.1
    pose.position.z += 0.2
    pose.orientation = Quaternion(q_tf[0], q_tf[1], q_tf[2], q_tf[3])

  print name, end_effector, ': pose_targets 2 (poses)', poses

  group.set_pose_targets(poses, end_effector)
  result = group.go()
  print name, ': end of pose (result)', result

  return result

def main(params):
  print 'main:', params

  scene = PlanningSceneInterface()
  robot = RobotCommander()
  rospy.sleep(1)

  create_objects(scene, robot)

  go_to_standing(robot, 'iiwa_two_robots')
  rospy.sleep(1)
  go_to_home(robot, 'iiwa_two_robots')
  rospy.sleep(1)

  objs = scene.get_objects()
  if 'part1' in objs:
    poses = objs['part1'].primitive_poses
    if pose_targets(robot, params[0]['name'], params[0]['end_effector'], poses) in (
            MoveItErrorCodes.SUCCESS,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ):
      print 'continue'
    else:
      go_to_home(robot, params[0]['name'])
      rospy.sleep(3)
      go_to_standing(robot, 'iiwa_two_robots')
      return

    rospy.sleep(3)
    go_to_home(robot, params[0]['name'])

  objs = scene.get_objects()
  if 'part2' in objs:
    poses = objs['part2'].primitive_poses
    if pose_targets(robot, params[1]['name'], params[1]['end_effector'], poses) in (
            MoveItErrorCodes.SUCCESS,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ):
      print 'continue'
    else:
      go_to_home(robot, params[1]['name'])
      rospy.sleep(3)
      go_to_standing(robot, 'iiwa_two_robots')
      return

    rospy.sleep(3)
    go_to_home(robot, params[1]['name'])

  """
  rospy.sleep(1)
  go_to_home(robot, 'iiwa_two_robots')
  """
  rospy.sleep(1)
  go_to_standing(robot, 'iiwa_two_robots')

  return

if __name__ == '__main__':
  roscpp_initialize(sys.argv)
  rospy.init_node('moveit_trial', anonymous=True)

  params = [
    { 'name': 'a_iiwa', 'end_effector' : '' },
    { 'name': 'b_iiwa', 'end_effector' : '' },
  ]

  if len(sys.argv) > 1:
    params[0]['name'] = sys.argv[1]
  if len(sys.argv) > 2:
    params[0]['end_effector'] = sys.argv[2]
  if len(sys.argv) > 3:
    params[1]['name'] = sys.argv[3]
  if len(sys.argv) > 4:
    params[1]['end_effector'] = sys.argv[4]

  main(params)
  print '----- end of main -----'

  rospy.spin()
  roscpp_shutdown()

