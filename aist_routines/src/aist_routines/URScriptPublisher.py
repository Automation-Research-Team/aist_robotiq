import os, sys, rospkg, rospy, tf
import std_msgs.msg
import ur_msgs.msg
from tf import transformations as tfs

######################################################################
#  class URScriptPublisher                                           #
######################################################################
class URScriptPublisher(object):
    def __init__(self, robot_name):
        super(URScriptPublisher, self).__init__()

        self._rospack    = rospkg.RosPack()
        self._robot_name = robot_name
        self._listener   = tf.TransformListener()
        self._publisher  = rospy.Publisher("/" + robot_name +
                                           "_controller/ur_driver/URScript",
                                           std_msgs.msg.String, queue_size=1)
        self._movej_template \
            = self._read_template("movej.script")
        self._movel_template \
            = self._read_template("movel.script")
        self._movel_rel_template \
            = self._read_template("movel_rel.script")
        self._linear_push_template \
            = self._read_template("linear_search_short.script")
        self._spiral_motion_template \
            = self._read_template("spiral_motion.script")
        self._insertion_template \
            = self._read_template("peginholespiral_imp_osx.script")
        self._horizontal_insertion_template \
            = self._read_template("peginholespiral_imp_osx_y_negative.script")

    # Publish various scripts
    def movej(self, joint_positions, acceleration, velocity, wait):
        if len(joint_positions) != 6:
            rospy.logwarn("Joint pose vector not of the correct length")
            return False
        return self._publish_script(self._movej_template, wait,
                                    joint_positions[0], joint_positions[1],
                                    joint_positions[2], joint_positions[3],
                                    joint_positions[4], joint_positions[5],
                                    acceleration, velocity)

    def movel(self, target_pose, end_effector_link,
              acceleration, velocity, wait):
        pose = self._pose_in_base_frame(target_pose, end_effector_link)
        if pose is None:
            return False

        # This seems to work, but it uses the ee_link TCP of the robot.
        return self._publish_script(self._movel_template, wait,
                                    pose[0][0], pose[0][1], pose[0][2],
                                    pose[1][0], pose[1][1], pose[1][2],
                                    acceleration, velocity)

    def movel_rel(self, translation, acceleration, velocity, wait):
        return self._publish_script(self._movel_rel_template, wait,
                                    translation.x, translation.y,
                                    translation.z, acceleration, velocity)

    def linear_push(self, max_force, force_direction,
                    max_approach_distance, forward_speed, wait):
        return self._publish_script(self._linear_push_template, wait,
                                    force_direction, max_force, forward_speed,
                                    max_approach_distance)

    def spiral_motion(self, acceleration, velocity, max_radius,
                      radius_increment, theta_increment, spiral_axis, wait):
        if (radius_increment < 0.0001) or (radius_increment > 0.005):
            rospy.logerr("radius_incr needs to be between 0.0001 and 0.005 but is " + str(radius_increment))
            return False
        return self._publish_script(self._spiral_motion_template, wait,
                                    max_radius, radius_increment,
                                    velocity, acceleration,
                                    spiral_axis, theta_increment)

    def insertion(self, max_force, force_direction, forward_speed,
                  max_approach_distance, max_radius, radius_increment,
                  max_insertion_distance, impedance_mass, peck_mode, wait):
        ### Function definitions, for reference:
        ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
        ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):
        return self._publish_script(self._insertion_template, wait,
                                    force_direction, max_force, forward_speed,
                                    max_approach_distance,
                                    max_insertion_distance,
                                    max_radius, radius_increment, peck_mode,
                                    impedance_mass)

    def horizontal_insertion(self, max_force, force_direction, forward_speed,
                             max_approach_distance, max_radius,
                             radius_increment, max_insertion_distance,
                             impedance_mass, peck_mode, wait):
        ### Function definitions, for reference:
        ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
        ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):
        return self._publish_script(self._horizontal_insertion_template, wait,
                                    force_direction, max_force, forward_speed,
                                    max_approach_distance,
                                    max_insertion_distance,
                                    max_radius, radius_increment, peck_mode,
                                    impedance_mass)

    # Private functions
    def _read_template(self, filename):
        with open(os.path.join(self._rospack.get_path("aist_routines"),
                               "src/aist_routines/urscript", filename),
                  'rb') as file:
            template = ""
            program_line = file.read(1024)
            while program_line:
                template += program_line
                program_line = file.read(1024)
            return template

    def _pose_in_base_frame(self, target_pose, end_effector_link):
        try:
            # Transformation from UR effecor frame to our effector_frame.
            eM0 = self._listener.fromTranslationRotation(
                    *self._listener.lookupTransform(
                        end_effector_link, self._robot_name + "_tool0",
                        rospy.Time(0)))

            # Convert target_pose to pose w.r.t. robot base_frame.
            base = self._robot_name + "_base"
            self._listener.waitForTransform(base, target_pose.header.frame_id,
                                            rospy.Time.now(),
                                            rospy.Duration(10))
            pose = self._listener.transformPose(base, target_pose).pose
            bMt  = self._listener.fromTranslationRotation(
                    (pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w))

            # Compute pose of the UR effector_frame w.r.t. robot base frame.
            bM0 = tfs.concatenate_matrices(bMt, eM0)

            return (tfs.translation_from_matrix(bM0),
                    tfs.euler_from_quaternion(tfs.quaternion_from_matrix(bM0)))

        except Exception as e:
            rospy.logerr("URScriptPublisher._pose_in_base_frame(): {}"
                         .format(e))
            return None


    def _publish_script(self, template, wait, *args):
        rospy.loginfo("Sending UR robot program ")
        # rospy.logdebug("Program is:")
        # rospy.logdebug(program)
        program_msg = std_msgs.msg.String()
        program_msg.data = template.format(*args)
        print(program_msg.data)
        self._publisher.publish(program_msg)
        if wait:
            return self._wait(rospy.Duration.from_sec(30.0))
        return True

    def _wait(self, timeout_duration=rospy.Duration.from_sec(20.0)):
        rospy.logdebug("Waiting for UR program to finish.")
        # Only run this after sending custom URScripts and not the regular
        # motion commands, or this call will not terminate before the timeout.
        rospy.sleep(1.0)
        t_start     = rospy.Time.now()
        time_passed = rospy.Time.now() - t_start
        while self._is_program_running():
            rospy.sleep(.05)
            time_passed = rospy.Time.now() - t_start
            if time_passed > timeout_duration:
                rospy.loginfo("Timeout reached.")
                return False
        rospy.logdebug("UR Program has terminated.")
        return True

    def _is_program_running(self):
        msg = rospy.wait_for_message("/" + self._robot_name +
                                     "_controller/ur_driver/robot_mode_state",
                                     ur_msgs.msg.RobotModeDataMsg)
        if msg:
            return msg.is_program_running
        else:
            rospy.logerr("No message received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
            return False
