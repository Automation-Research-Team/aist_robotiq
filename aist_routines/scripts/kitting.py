#!/usr/bin/env python

import rospy, collections
import aist_routines.base as base
from aist_routines.base import AISTBaseRoutines
from aist_routines      import msg as amsg

######################################################################
#  class KittingRoutines                                             #
######################################################################
class KittingRoutines(AISTBaseRoutines):
    """Implements kitting routines for aist robot system."""

    Bin = collections.namedtuple('Bin', 'name part_id part_props')

    def __init__(self):
        super(KittingRoutines, self).__init__()

        bin_props  = base.paramtuples(rospy.get_param('~bin_props'))
        part_props = base.paramtuples(rospy.get_param('~part_props'))

        # Assign part information to each bin.
        self._bins = {}
        for bin_id, bin_prop in bin_props.items():
            part_id = bin_prop.part_id
            self._bins[bin_id] = KittingRoutines.Bin(bin_prop.name, part_id,
                                                     part_props[part_id])

        self._former_robot_name = None
        self._fail_poses = []
        #self.go_to_named_pose('all_bots', 'home')

    @property
    def nbins(self):
        return len(self._bins)

    @property
    def former_robot_name(self):
        return self._former_robot_name

    ###----- main procedure
    def run(self):
        self.go_to_named_pose('all_bots', 'back')
        for bin in self._bins:
            if rospy.is_shutdown():
                break
            self.attempt_bin(bin, 1)
        self.go_to_named_pose('all_bots', 'home')

    def demo(self):
        bin_ids = (1, 4, 5)

        while True:
            completed = False

            for bin_id in bin_ids:
                kitting.clear_fail_poses()
                success = kitting.attempt_bin(bin_id, 5, 0)
                completed = completed and not success

            if completed:
                break

        kitting.go_to_named_pose(kitting.former_robot_name, 'home')

    def search(self, bin_id, marker_lifetime=0):
        bin   = self._bins[bin_id]
        props = bin.part_props
        self.graspability_send_goal(props.robot_name, props.camera_name,
                                    bin.part_id, bin_id)
        return self.graspability_wait_for_result(marker_lifetime)
        # return self.search_graspability(props.robot_name, props.camera_name,
        #                                 bin.part_id, bin_id, props.use_normals,
        #                                 marker_lifetime)

    def attempt_bin(self, bin_id, max_attempts=5, marker_lifetime=0):
        bin   = self._bins[bin_id]
        props = bin.part_props

        # If using a different robot from the former, move it back to home.
        if self._former_robot_name is not None and \
           self._former_robot_name != props.robot_name:
            self.go_to_named_pose(self._former_robot_name, 'back')
        self._former_robot_name = props.robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(props.robot_name, props.camera_name):
            self.go_to_frame(props.robot_name, bin.name, (0, 0, 0.15))

        # Search for graspabilities.
        (pick_poses, _, success) = self.search(bin_id, marker_lifetime)
        if not success:
            return False

        # Attempt to pick the item.
        nattempts = 0
        for pose in pick_poses:
            if nattempts == max_attempts:
                break

            if self._is_close_to_fail_poses(pose):
                continue

            result = self.pick(props.robot_name, pose,
                               speed_slow=props.speed_slow,
                               grasp_offset=props.grasp_offset,
                               approach_offset=props.approach_offset)
            if result == amsg.pickOrPlaceResult.SUCCESS:
                result = self.place_at_frame(
                                props.robot_name, props.destination,
                                speed_slow=props.speed_slow,
                                place_offset=props.place_offset,
                                approach_offset=props.approach_offset)
                return result == amsg.pickOrPlaceResult.SUCCESS
            elif result == amsg.pickOrPlaceResult.MOVE_FAILURE or \
                 result == amsg.pickOrPlaceResult.APPROACH_FAILURE:
                self._fail_poses.append(pose)
            elif result == amsg.pickOrPlaceResult.DEPARTURE_FAILURE:
                self.release(props.robot_name)
                raise RuntimeError('Failed to depart from pick/place pose')
            elif result == amsg.pickOrPlaceResult.PICK_FAILURE:
                self._fail_poses.append(pose)
                nattempts += 1

            self.release(props.robot_name)

        return False

    def clear_fail_poses(self):
        self._fail_poses = []

    def _is_eye_on_hand(self, robot_name, camera_name):
        return camera_name == robot_name + '_camera'

    def _is_close_to_fail_poses(self, pose):
        for fail_pose in self._fail_poses:
            if self._is_close_to_fail_pose(pose, fail_pose, 0.005):
                return True
        return False

    def _is_close_to_fail_pose(self, pose, fail_pose, tolerance):
        position      = pose.pose.position
        fail_position = fail_pose.pose.position
        if abs(position.x - fail_position.x) > tolerance or \
           abs(position.y - fail_position.y) > tolerance or \
           abs(position.z - fail_position.z) > tolerance:
            return False
        return True


if __name__ == '__main__':

    rospy.init_node('kitting', anonymous=True)

    with KittingRoutines() as kitting:
        while not rospy.is_shutdown():
            print('============ Kitting procedures ============ ')
            print('  b: Create a backgroud image')
            print('  m: Create a mask image')
            print('  s: Search graspabilities')
            print('  a: Attempt to pick and place')
            print('  A: Repeat attempts to pick and place')
            print('  d: Perform small demo')
            print('  k: Do kitting task')
            print('  g: Grasp')
            print('  r: Release')
            print('  H: Move all robots to home')
            print('  B: Move all robots to back')
            print('  q: Quit')

            try:
                key = raw_input('>> ')
                if key == 'q':
                    if kitting.former_robot_name is not None:
                        kitting.go_to_named_pose(kitting.former_robot_name,
                                                 'home')
                    break
                elif key == 'H':
                    kitting.go_to_named_pose('all_bots', 'home')
                elif key == 'B':
                    kitting.go_to_named_pose('all_bots', 'back')
                elif key == 'b':
                    kitting.create_background_image('a_phoxi_m_camera')
                elif key == 'm':
                    kitting.create_mask_image('a_phoxi_m_camera',
                                              kitting.nbins)
                elif key == 's':
                    bin_id = int(raw_input('  bin id? '))
                    kitting.search(bin_id)
                elif key == 'a':
                    bin_id = int(raw_input('  bin id? '))
                    kitting.clear_fail_poses()
                    kitting.attempt_bin(bin_id, 5, 0)
                    kitting.go_to_named_pose(kitting.former_robot_name, 'home')
                elif key == 'A':
                    bin_id = int(raw_input('  bin id? '))
                    kitting.clear_fail_poses()
                    while kitting.attempt_bin(bin_id, 5, 0):
                        pass
                    kitting.go_to_named_pose(kitting.former_robot_name, 'home')
                elif key == 'd':
                    kitting.demo()
                elif key == 'k':
                    kitting.run()
                elif key == 'g':
                    if kitting.former_robot_name is not None:
                        kitting.grasp(kitting.former_robot_name)
                elif key == 'r':
                    if kitting.former_robot_name is not None:
                        kitting.release(kitting.former_robot_name)
            except Exception as e:
                print(e.message)
