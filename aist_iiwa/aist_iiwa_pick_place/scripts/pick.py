#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, Grasp
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
import tf

z_ext = 0.80

def get_prefix(name):
  prefix = 'iiwa'
  if 'a_iiwa' in name:
    prefix = 'a_iiwa'
  elif 'b_iiwa' in name:
    prefix = 'b_iiwa'
  return prefix

def get_joint_names(name, end_effector):
  joint_names = {
    'bh282' : [
                'bh_j11_joint',
                'bh_j12_joint',
                'bh_j22_joint',
                'bh_j32_joint'
    ],
    'r85g' : [
                'robotiq_85_left_knuckle_joint'
    ]
  }
  result = []
  for _name in joint_names[end_effector]:
    result.append(name + '_' + _name)
  return result

def get_grasp_positions(end_effector):
  grasp_positions = {
    'bh282' : [
        [ 0, 0, 0, 0 ],
        [ 0.5, 1.5, 1.5, 1.25 ]
    ],
    'r85g' : [
        [ 0 ],
        [ 0.7 ]
    ]
  }
  return grasp_positions[end_effector]

def get_correction_value(end_effector):
  correction_value = {
    'bh282' : 0.15,
    'r85g'  : 0.085
  }
  return correction_value[end_effector]


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
  scene.add_box("part", p, (0.03, 0.03, 0.3))

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

def pick_from_table(robot, name, end_effector, target):
  print name, end_effector, ': pick_from_table (target)', target

  group = robot.get_group(name)
  prefix = get_prefix(name)

  link_0 = robot.get_link(prefix + '_link_0')
  link_0_pose = link_0.pose()

  joint_names = get_joint_names(prefix, end_effector)

  grasp_positions = get_grasp_positions(end_effector)
  correction_value = get_correction_value(end_effector)

  grasp = Grasp()

  # grasp.grasp_pose.header.frame_id = prefix + '_link_0'
  grasp.grasp_pose.header.frame_id = robot.get_planning_frame()
  grasp.grasp_pose.pose.position.x = target.pose.position.x + (
        -1 * correction_value
        if target.pose.position.x > link_0_pose.pose.position.x
        else correction_value)
  grasp.grasp_pose.pose.position.y = target.pose.position.y
  grasp.grasp_pose.pose.position.z = target.pose.position.z + 0.1

  q_tf = tf.transformations.quaternion_from_euler(0, 3.14/2, 0)
  grasp.grasp_pose.pose.orientation = Quaternion(q_tf[0], q_tf[1], q_tf[2], q_tf[3])

  grasp.pre_grasp_approach.direction.header.frame_id = prefix + '_link_0'
  grasp.pre_grasp_approach.direction.vector.x = 1
  grasp.pre_grasp_approach.min_distance = 0.1
  grasp.pre_grasp_approach.desired_distance = 0.15

  grasp.post_grasp_retreat.direction.header.frame_id = prefix + '_link_0'
  grasp.post_grasp_retreat.direction.vector.z = 1
  grasp.post_grasp_retreat.min_distance = 0.1
  grasp.post_grasp_retreat.desired_distance = 0.2


  grasp.pre_grasp_posture.header.frame_id = prefix + '_link_0'
  grasp.pre_grasp_posture.joint_names = joint_names
  jtp = JointTrajectoryPoint()
  jtp.positions = grasp_positions[0]
  jtp.time_from_start.secs = 1
  grasp.pre_grasp_posture.points = [ jtp ]

  grasp.grasp_posture.header.frame_id = prefix + '_link_0'
  grasp.grasp_posture.joint_names = joint_names
  jtp = JointTrajectoryPoint()
  jtp.positions = grasp_positions[1]
  jtp.time_from_start.secs = 1
  grasp.grasp_posture.points = [ jtp ]

  rospy.sleep(1)

  result = group.pick("part", [ grasp ])
  print name, ': end of pick (result)', result

  return result

def place_on_table(robot, name, x, y, z, i, j, k):
  print name, ': place_on_table (x, y, z, i, j, k)', x, y, z, i, j, k

  group = robot.get_group(name)

  location = PoseStamped()
  location.header.frame_id = robot.get_planning_frame()
  location.pose.position.x = x
  location.pose.position.y = y
  location.pose.position.z = z
  q_tf = tf.transformations.quaternion_from_euler(i, j, k)
  location.pose.orientation = Quaternion(q_tf[0], q_tf[1], q_tf[2], q_tf[3])

  result = group.place("part", location)
  print name, ': end of place (result)', result

  return result

def main(params):
  print 'main:', params

  scene = PlanningSceneInterface()
  robot = RobotCommander()
  rospy.sleep(1)

  create_objects(scene, robot)

  go_to_standing(robot, params[2]['name'])
  rospy.sleep(3)
  """
  go_to_home(robot, params[2]['name'])
  rospy.sleep(3)
  """

  objs = scene.get_objects()
  if 'part' in objs:
    target = PoseStamped()
    target.header.frame_id = objs['part'].header.frame_id
    target.pose = objs['part'].primitive_poses[0]
    if pick_from_table(robot, params[0]['name'], params[0]['end_effector'], target) in (
            MoveItErrorCodes.SUCCESS,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ):
      print 'continue'
    else:
      go_to_home(robot, params[0]['name'])
      rospy.sleep(3)
      go_to_standing(robot, params[2]['name'])
      return
    rospy.sleep(3)
    if place_on_table(robot, params[0]['name'], 0.5, 0.6, 0.5+z_ext, 0.0, 0.0, 0.0) is False:
      go_to_home(robot, params[0]['name'])
      rospy.sleep(3)
      go_to_standing(robot, params[2]['name'])
      return
    rospy.sleep(1)
    go_to_home(robot, params[0]['name'])
    rospy.sleep(1)

  objs = scene.get_objects()
  if 'part' in objs:
    target = PoseStamped()
    target.header.frame_id = objs['part'].header.frame_id
    target.pose = objs['part'].primitive_poses[0]
    if pick_from_table(robot, params[1]['name'], params[1]['end_effector'], target) in (
            MoveItErrorCodes.SUCCESS,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ):
      print 'continue'
    else:
      go_to_home(robot, params[1]['name'])
      rospy.sleep(3)
      go_to_standing(robot, params[2]['name'])
      return
    rospy.sleep(3)
    link_0 = robot.get_link('b_iiwa_link_0')
    link_0_pose = link_0.pose()
    if place_on_table(robot, params[1]['name'],
            0.8, link_0_pose.pose.position.y, 0.5+z_ext, 0.0, 0.0, 0.0) is False:
      go_to_home(robot, params[1]['name'])
      rospy.sleep(3)
      go_to_standing(robot, params[2]['name'])
      return
    rospy.sleep(1)
    go_to_home(robot, params[1]['name'])

  """
  rospy.sleep(3)
  go_to_home(robot, params[2]['name'])
  """
  rospy.sleep(3)
  go_to_standing(robot, params[2]['name'])

  return

if __name__ == '__main__':
  roscpp_initialize(sys.argv)
  rospy.init_node('moveit_trial', anonymous=True)

  """
  params = [
    { 'name': 'a_iiwa_arm', 'end_effector' : 'bh282' },
    { 'name': 'b_iiwa_arm', 'end_effector' : 'bh282' },
    { 'name': 'iiwa_two_robots_groups', 'end_effector' : 'bh282' },
  ]
  """
  params = [
    { 'name': 'a_iiwa', 'end_effector' : 'bh282' },
    { 'name': 'b_iiwa', 'end_effector' : 'bh282' },
    { 'name': 'iiwa_two_robots_groups', 'end_effector' : 'bh282' },
  ]

  if len(sys.argv) > 1:
    params[0]['name'] = sys.argv[1]
  if len(sys.argv) > 2:
    params[0]['end_effector'] = sys.argv[2]
  if len(sys.argv) > 3:
    params[1]['name'] = sys.argv[3]
  if len(sys.argv) > 4:
    params[1]['end_effector'] = sys.argv[4]
  if len(sys.argv) > 5:
    params[2]['name'] = sys.argv[5]
  if len(sys.argv) > 6:
    params[2]['end_effector'] = sys.argv[6]

  main(params)
  print '----- end of main -----'

  rospy.spin()
  roscpp_shutdown()

