#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, Grasp
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Quaternion
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
  scene.add_box("part", p, (0.03, 0.03, 0.3))

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

def pick_from_table(robot, robot_name, target):
  print robot_name, ': pick_from_table (target)', target

  link_0 = robot.get_link(robot_name + '_link_0')
  link_0_pose = link_0.pose()

  grasp = Grasp()

  # grasp.grasp_pose.header.frame_id = robot_name + '_link_0'
  grasp.grasp_pose.header.frame_id = robot.get_planning_frame()
  grasp.grasp_pose.pose.position.x = target.pose.position.x + (
        -0.085 if target.pose.position.x > link_0_pose.pose.position.x else 0.085)
  grasp.grasp_pose.pose.position.y = target.pose.position.y
  grasp.grasp_pose.pose.position.z = target.pose.position.z + 0.1

  q_tf = tf.transformations.quaternion_from_euler(0, 3.14/2, 0)
  grasp.grasp_pose.pose.orientation = Quaternion(q_tf[0], q_tf[1], q_tf[2], q_tf[3])

  grasp.pre_grasp_approach.direction.header.frame_id = robot_name + '_link_0'
  grasp.pre_grasp_approach.direction.vector.x = 1
  grasp.pre_grasp_approach.min_distance = 0.1
  grasp.pre_grasp_approach.desired_distance = 0.15

  grasp.post_grasp_retreat.direction.header.frame_id = robot_name + '_link_0'
  grasp.post_grasp_retreat.direction.vector.z = 1
  grasp.post_grasp_retreat.min_distance = 0.1
  grasp.post_grasp_retreat.desired_distance = 0.2

  grasp.pre_grasp_posture.header.frame_id = robot_name + '_link_0'
  grasp.pre_grasp_posture.joint_names = [
                robot_name + '_r85grobotiq_85_left_knuckle_joint'
                ]
  jtp = JointTrajectoryPoint()
  jtp.positions = [ 0 ]
  jtp.time_from_start.secs = 1
  grasp.pre_grasp_posture.points = [ jtp ]

  grasp.grasp_posture.header.frame_id = robot_name + '_link_0'
  grasp.grasp_posture.joint_names = [
                robot_name + '_r85grobotiq_85_left_knuckle_joint'
                ]
  jtp = JointTrajectoryPoint()
  jtp.positions = [ 0.7 ]
  jtp.time_from_start.secs = 1
  grasp.grasp_posture.points = [ jtp ]

  rospy.sleep(1)

  result = False

  if robot_name is 'a_iiwa':
    result =robot.a_iiwa_arm.pick("part", [ grasp ])
  elif robot_name is 'b_iiwa':
    result =robot.b_iiwa_arm.pick("part", [ grasp ])
  elif robot_name is 'iiwa':
    result =robot.iiwa_arm.pick("part", [ grasp ])
  else:
    print robot_name, 'is unknown.'

  print robot_name, ': end of pick (result)', result
  return result

def place_on_table(robot, robot_name, x, y, z, i, j, k):
  print robot_name, ': place_on_table (x, y, z, i, j, k)', x, y, z, i, j, k

  location = PoseStamped()
  location.header.frame_id = robot.get_planning_frame()
  location.pose.position.x = x
  location.pose.position.y = y
  location.pose.position.z = z
  q_tf = tf.transformations.quaternion_from_euler(i, j, k)
  location.pose.orientation = Quaternion(q_tf[0], q_tf[1], q_tf[2], q_tf[3])

  result = False

  if robot_name is 'a_iiwa':
    result = robot.a_iiwa_arm.place("part", location)
  elif robot_name is 'b_iiwa':
    result = robot.b_iiwa_arm.place("part", location)
  elif robot_name is 'iiwa':
    result = robot.iiwa_arm.place("part", location)
  else:
    print robot_name, 'is unknown.'

  print robot_name, ': end of place (result)', result
  return result

def main():
  scene = PlanningSceneInterface()
  robot = RobotCommander()
  rospy.sleep(1)

  create_objects(scene, robot)

  objs = scene.get_objects()
  if 'part' in objs:
    target = PoseStamped()
    target.header.frame_id = objs['part'].header.frame_id
    target.pose = objs['part'].primitive_poses[0]
    if pick_from_table(robot, 'a_iiwa', target) in (
            MoveItErrorCodes.SUCCESS,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ):
      print 'continue'
    else:
      go_to_home(robot, 'a_iiwa')
      return
    rospy.sleep(3)
    if place_on_table(robot, 'a_iiwa', 0.5, 0.6, 0.5+z_ext, 0.0, 0.0, 0) is False:
      go_to_home(robot, 'a_iiwa')
      return
    rospy.sleep(1)
    go_to_home(robot, 'a_iiwa')

  objs = scene.get_objects()
  if 'part' in objs:
    target = PoseStamped()
    target.header.frame_id = objs['part'].header.frame_id
    target.pose = objs['part'].primitive_poses[0]
    if pick_from_table(robot, 'b_iiwa', target) in (
            MoveItErrorCodes.SUCCESS,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ):
      print 'continue'
    else:
      go_to_home(robot, 'b_iiwa')
      return
    rospy.sleep(3)
    link_0 = robot.get_link('b_iiwa_link_0')
    link_0_pose = link_0.pose()
    if place_on_table(robot, 'b_iiwa',
            0.8, link_0_pose.pose.position.y, 0.5+z_ext, 0.0, 0.0, 0.0) is False:
      go_to_home(robot, 'b_iiwa')
      return
    rospy.sleep(1)
    go_to_home(robot, 'b_iiwa')

  return

if __name__ == '__main__':
  roscpp_initialize(sys.argv)
  rospy.init_node('moveit_r85g', anonymous=True)

  main()
  print '----- end of main -----'

  rospy.spin()
  roscpp_shutdown()

