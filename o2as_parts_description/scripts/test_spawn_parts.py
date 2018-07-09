#!/usr/bin/python

import argparse
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import *

urdf_dir = '/root/catkin_ws/src/o2as_parts_description/urdf/'
urdf_files = {
    4: urdf_dir + '/04_37D-GEARMOTOR-50-70.urdf',
    5: urdf_dir + '/05_MBRFA30-2-P6.urdf',
    6: urdf_dir + '/06_MBT4-400.urdf',
    7: urdf_dir + '/07_SBARB6200ZZ_30.urdf',
    8: urdf_dir + '/08_KZAF1075NA4WA55GA20AA0.urdf',
    9: urdf_dir + '/09_EDCS10.urdf',
    10: urdf_dir + '/10_CLBPS10_17_4.urdf',
    11: urdf_dir + '/11_MBRAC60-2-10.urdf',
    12: urdf_dir + '/12_CLBUS6-9-9.5.urdf',
    13: urdf_dir + '/13_MBGA30-2.urdf',
    14: urdf_dir + '/14_BGPSL6-9-L30-F8.urdf',
    15: urdf_dir + '/15_SLBNR6.urdf',
    16: urdf_dir + '/16_SPWF6.urdf',
    17: urdf_dir + '/17_SCB4-10.urdf',
    18: urdf_dir + '/18_SCB3-10.urdf',
}


def spawn_part(ix_part):
    # Create service proxy
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    rospy.wait_for_service('/gazebo/spawn_urdf_model')

    # Load URDF
    with open(urdf_files[ix_part], "r") as f:
        model_xml = f.read()

    # Object pose
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose = Pose()
    pose.position.x = 0.25
    pose.position.y = 0
    pose.position.z = 2
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    # Spawn model
    req = SpawnModelRequest()
    req.model_name = 'part_{}'.format(ix_part)
    req.initial_pose = pose
    req.model_xml = model_xml
    req.robot_namespace = '/' + str(ix_part)
    req.reference_frame = 'world'
    spawn_model(req)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Spawn workpiece')
    parser.add_argument('ix_part', metavar='ix_part', type=int,
                        help='Parts number, integer between 4 and 18')
    args = parser.parse_args()
    print(args.ix_part)

    spawn_part(args.ix_part)
