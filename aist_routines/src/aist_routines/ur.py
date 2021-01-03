import rospy
from aist_routines     import AISTBaseRoutines
from URScriptPublisher import URScriptPublisher

######################################################################
#  class URRoutines                                                  #
######################################################################
class URRoutines(AISTBaseRoutines):
    def __init__(self):
        super(URRoutines, self).__init__()

        if rospy.get_param("use_real_robot", False):
            self._urscript_publishers = {
                "a_bot": URScriptPublisher("a_bot"),
                "b_bot": URScriptPublisher("b_bot"),
                "c_bot": URScriptPublisher("c_bot"),
                "d_bot": URScriptPublisher("d_bot"),
            }

    # UR script motions
    def ur_movej(self, robot_name,
                 joint_positions, acceleration=0.5, velocity=0.5, wait=True):
        pub = self._urscript_publishers[robot_name]
        return pub.movej(self, joint_positions, acceleration, velocity, wait)

    def ur_movel(self, robot_name, target_pose, end_effector_link="",
                 acceleration=0.5, velocity=0.03, wait=True):
        pub = self._urscript_publishers[robot_name]
        if end_effector_link == "":
            end_effector_link = self._grippers[robot_name].tip_link
        success = pub.movel(target_pose, end_effector_link,
                            acceleration, velocity, wait)
        current_pose = self.get_current_pose(robot_name)
        is_all_close = self._all_close(target_pose, current_pose, 0.01)
        return (success, is_all_close, current_pose)

    def ur_movel_rel(self, robot_name, translation,
                     acceleration=0.5, velocity=0.03, wait=True):
        pub = self._urscript_publishers[robot_name]
        return pub.movel_rel(translation, acceleration, velocity, wait)

    def ur_linear_push(self, robot_name, force=10.0, wait=True, direction="Z+",
                       max_approach_distance=0.1, forward_speed=0.02):
        pub = self._urscript_publishers[robot_name]
        return pub.linear_push(force, direction,
                               max_approach_distance, forward_speed, wait)

    def ur_spiral_motion(self, robot_name, acceleration=0.1, velocity=0.03,
                         max_radius=0.0065, radius_increment=0.002,
                         theta_increment=30, spiral_axis="Z", wait=True):
        pub = self._urscript_publishers[robot_name]
        return pub.spiral_motion(acceleration, velocity,
                                 max_radius, radius_increment,
                                 theta_increment, spiral_axis, wait)

    def ur_insertion(self, robot_name,
                     max_force=10.0, force_direction="Z+",
                     forward_speed=0.02, max_approach_distance=0.1,
                     max_radius=0.004, radius_increment=0.0003,
                     max_insertion_distance=0.035, impedance_mass=10,
                     peck_mode=False, wait=True):
        pub = self._urscript_publishers[robot_name]
        return pub.insertion(max_force, force_direction,
                             forward_speed, max_approach_distance,
                             max_radius, radius_increment,
                             max_insertion_distance, impedance_mass,
                             peck_mode, wait)

    def ur_horizontal_insertion(self, robot_name,
                                max_force=10.0, force_direction="Y-",
                                forward_speed=0.02, max_approach_distance=0.1,
                                max_radius=0.007, radius_increment=0.0003,
                                max_insertion_distance=0.035,
                                impedance_mass=10,
                                peck_mode=False, wait=True):
        pub = self._urscript_publishers[robot_name]
        return pub.horizontal_insertion(max_force, force_direction,
                                        forward_speed, max_approach_distance,
                                        max_radius, radius_increment,
                                        max_insertion_distance,
                                        impedance_mass, peck_mode, wait)
