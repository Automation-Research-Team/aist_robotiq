#!/usr/bin/env python

import rospy, argparse
from aist_localization  import LocalizationClient
from aist_model_spawner import ModelSpawnerClient

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

    spawner = ModelSpawnerClient()

    for pose, overlap in zip(poses, overlaps):
        print("{}\noverlap: {}".format(pose, overlap))
        spawner.add(args.model, pose)

    print(spawner.get_list())
