#!/usr/bin/env python

import os, csv, copy, time, datetime, re, collections
import rospy, rospkg, rosparam
import aist_routines.base as base
from aist_routines.fetch import FetchRoutines
from aist_routines       import msg as amsg

######################################################################
#  global variables                                                  #
######################################################################
rp = rospkg.RosPack()

ts = time.time()
start_date_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
number_of_attempted = 1

######################################################################
#  class CEATECDemoRoutines                                          #
######################################################################
class CEATECDemoRoutines(FetchRoutines):
    """Implements kitting routines for aist robot system."""


    @staticmethod
    def paramtuples(d):
        fields = set()
        for params in d.values():
            for field in params.keys():
                fields.add(field)
        ParamTuple = collections.namedtuple("ParamTuple", " ".join(fields))

        params = {}
        for key, param in d.items():
            params[key] = ParamTuple(**param)
        return params

    def __init__(self):
        super(CEATECDemoRoutines, self).__init__()

        rospack = rospkg.RosPack()
        d = rosparam.load_file(rospack.get_path("aist_routines") +
                               "/config/ceatec_demo.yaml")[0][0]
        self._item_props = CEATECDemoRoutines.paramtuples(d["item_props"])

    @property
    def nitems(self):
        return len(self._item_props)

    def item_props(self, item_name):
        return self._item_props[item_name]

    ###----- main procedure
    def pick_item_at_frame(self, item, frame):
        print("### pick_item_at_frame() ###")
        props = self._item_props[item]
        print("props = {}".format(props))
        return self.pick_at_frame(props.robot_name, frame,
                                  grasp_offset=props.grasp_offset,
                                  approach_offset=props.approach_offset,
                                  speed_slow=props.speed_slow)

    def place_item_at_frame(self, item, frame):
        props = self._item_props[item]
        print("props = {}".format(props))
        return self.place_at_frame(props.robot_name, frame,
                                   place_offset=props.place_offset,
                                   approach_offset=props.approach_offset,
                                   speed_slow=props.speed_slow)

if __name__ == '__main__':
    with CEATECDemoRoutines() as demo:
        while not rospy.is_shutdown():
            print("============ CEATECDemo procedures ============ ")
            print("  b: Create a backgroud image")
            print("  m: Create a mask image")
            print("  s: Search graspabilities")
            print("  a: Attempt to pick and place")
            print("  A: Repeat attempts to pick and place")
            print("  k: Do demo task")
            print("  T: move to tucking pose")
            print("  R: move to ready pose")
            print("  P: Move to pick_ready pose")
            print("  q: Quit")

            try:
                key = raw_input(">> ")
                if key == 'q':
                    break
                elif key == 'T':
                    demo.go_to_named_pose("tucking", "arm")
                elif key == 'R':
                    demo.go_to_named_pose("ready", "arm")
                elif key == 'P':
                    demo.go_to_named_pose("pick_ready", "arm")
                elif key == 'b':
                    demo.create_background_image("head_camera")
                # elif key == 'm':
                #     demo.create_mask_image("head_camera", demo.nmasks)
                # elif key == 's':
                #     item  = raw_input("  item name? ")
                #     props = demo.props(item)
                #     demo.graspability_send_goal(props.robot_name,
                #                                 props.camera_name,
                #                                 item.part_id, item.bin_id,
                #                                 props.use_normals)
                #     demo.graspability_wait_for_result(props.camera_name,
                #                                       marker_lifetime=0)
                elif key == 'go':
                    demo.move_base_to_frame(raw_input(" frame? "))
                elif key == 'move_base':
                    x     = float(raw_input("  x     = "))
                    y     = float(raw_input("  y     = "))
                    theta = float(raw_input("  theta = "))
                    demo.move_base(x, y, theta)
                elif key == 'pick':
                    item  = raw_input("  item?  ")
                    frame = raw_input("  frame? ")
                    demo.pick_item_at_frame(item, frame)
                elif key == 'place':
                    item  = raw_input("  item?  ")
                    frame = raw_input("  frame? ")
                    demo.place_item_at_frame(item, frame)
            except Exception as e:
                print(e.message)
