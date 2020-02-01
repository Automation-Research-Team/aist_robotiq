#!/usr/bin/env python

import os, sys, rospy, rospkg, argparse
import actionlib
import xacro
from std_srvs.srv      import Trigger
from aist_localization import msg as lmsg
from operator          import itemgetter

#########################################################################
#  class URDFSapwner                                                    #
#########################################################################
class URDFSpawner(object):
    _template = ""
    def __init__(self, urdf_dir):
        super(URDFSpawner, self).__init__()

        self._urdf_dir = urdf_dir

    def add(self, pose, model, param=False):
        macro_file = os.path.join(self._urdf_dir, model + "_macro.urdf.xacro")
        doc = xacro.process_file(macro_file)
        sys.stdout.write(doc.toprettyxml(indent='  ', encoding='utf8'))

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

    rospack = rospkg.RosPack()
    urdf_dir = rospack.get_path("o2as_parts_description") + "/urdf/generated"

    spawner = URDFSpawner(urdf_dir)

    for pose, overlap in zip(poses, overlaps):
        print("{}\noverlap: {}".format(pose, overlap))
        spawner.add(pose, args.model)
