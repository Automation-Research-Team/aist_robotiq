#!/usr/bin/env python

import os, sys, rospy, rospkg, argparse, re
import actionlib
import xacro
from tf                import transformations as tfs
from std_srvs.srv      import Trigger
from aist_localization import msg as lmsg
from operator          import itemgetter

#########################################################################
#  class URDFSapwner                                                    #
#########################################################################
class URDFSpawner(object):

    _BeginRobot = "<robot name=\"{0}\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n"
    _Macro      = "  <xacro:include filename=\"{1}\"/>\n  <xacro:{0} prefix=\"{2}\" parent=\"{3}\" spawn_attached=\"true\">\n    <origin xyz=\"{4} {5} {6}\" rpy=\"{7} {8} {9}\"/>\n  </xacro:{0}>\n"
    _EndRobot   = "</robot>"

    def __init__(self, param_name="parts_description"):
        super(URDFSpawner, self).__init__()

        self._param_name = param_name
        self._macros     = {}

    def add(self, name, macro_file, pose):
        rpy = tfs.euler_from_quaternion([pose.pose.orientation.x,
                                         pose.pose.orientation.y,
                                         pose.pose.orientation.z,
                                         pose.pose.orientation.w])
        self._macros[name] = URDFSpawner._Macro.format(name, macro_file, "",
                                                       pose.header.frame_id,
                                                       pose.pose.position.x,
                                                       pose.pose.position.y,
                                                       pose.pose.position.z,
                                                       *rpy)
        self._spawn()

    def delete(self, name):
        del self._macros[name]
        if self._macros:
            self._spawn()
        else:
            self._delete_param()

    def delete_all(self):
        self._macros.clear()
        self._delete_param()

    def _spawn(self):
        # Create parts description in Xacro format.
        desc = URDFSpawner._BeginRobot.format(self._param_name)
        for macro in self._macros.values():
            desc += macro
        desc += URDFSpawner._EndRobot

        # Expand and process Xacro into XML format.
        doc = xacro.parse(desc)  # Create DOM tree.
        xacro.process_doc(doc)   # Expand and process macros.
        xml = doc.toprettyxml(indent='  ', encoding='utf8')

        rospy.set_param(self._param_name, xml)
        print(xml)

    def _delete_param(self):
        try:
            rospy.delete_param(self._param_name)
        except KeyError:
            rospy.logwarn("parameter[" + self._param_name + "] not existing.")


#########################################################################
#  class LocalizationClient                                             #
#########################################################################
class LocalizationClient(object):
    def __init__(self):
        super(LocalizationClient, self).__init__()

        self._load_scene \
            = rospy.ServiceProxy("localization/load_scene", Trigger)
        self._localize = actionlib.SimpleActionClient("localization/localize",
                                                      lmsg.localizeAction)
        self._localize.wait_for_server()

    def load_scene(self):
        return self._load_scene().success

    def send_goal(self, object_name, number_of_poses=1):
        self._poses    = []
        self._overlaps = []
        goal = lmsg.localizeGoal()
        goal.object_name     = object_name
        goal.number_of_poses = number_of_poses
        self._localize.send_goal(goal, feedback_cb=self._feedback_cb)

    def wait_for_result(self, timeout=5):
        if (not self._localize.wait_for_result(timeout)):
            self._localize.cancel_goal()  # Cancel goal if timeout expired.

        # Sort obtained poses in descending order of overlap values.
        pairs = sorted(zip(self._poses, self._overlaps),
                       key=itemgetter(1), reverse=True)
        if len(pairs):
            self._poses, self._overlaps = zip(*pairs)

        result = self._localize.get_result()
        return (self._poses, self._overlaps,
                result.success if result else False)

    def _feedback_cb(self, feedback):
        self._poses.append(feedback.pose)
        self._overlaps.append(feedback.overlap)


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test localizeAction')
    parser.add_argument('-m',
                        '--model',
                        action='store',
                        nargs='?',
                        default='04_37D-GEARMOTOR-50-70',
                        type=str,
                        choices=None,
                        help='name of model to be matched',
                        metavar=None)
    parser.add_argument('-n',
                        '--number_of_poses',
                        action='store',
                        nargs='?',
                        default=1,
                        type=int,
                        choices=None,
                        help='the number of candidate poses',
                        metavar=None)
    parser.add_argument('-t',
                        '--timeout',
                        action='store',
                        nargs='?',
                        default=5,
                        type=float,
                        choices=None,
                        help='timeout value',
                        metavar=None)
    args = parser.parse_args()

    rospy.init_node('localization_client')

    localize = LocalizationClient()
    localize.load_scene()
    localize.send_goal(args.model, args.number_of_poses)
    (poses, overlaps, success) \
        = localize.wait_for_result(rospy.Duration(args.timeout))

    rospack    = rospkg.RosPack()
    urdf_dir   = rospack.get_path("o2as_parts_description") + "/urdf/generated"
    macro_file = os.path.join(urdf_dir, args.model + "_macro.urdf.xacro")
    macro_name = "assy_part_" + re.split("[_-]", args.model)[0]

    spawner = URDFSpawner()

    for pose, overlap in zip(poses, overlaps):
        print("{}\noverlap: {}".format(pose, overlap))
        spawner.add(macro_name, macro_file, pose)
