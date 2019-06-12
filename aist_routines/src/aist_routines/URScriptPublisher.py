import os, sys, rospkg
import roslib
import rospy
import tf
import std_msgs.msg
import ur_msgs.msg

######################################################################
#  class URScriptPublisher                                           #
######################################################################
class URScriptPublisher(object):
    def __init__(self, robot_name):
        super(URScriptPublisher, self).__init__()
        self._robot_name = robot_name
        self._listener   = tf.TransformListener()
        self._publisher  = rospy.Publisher("/" + robot_name +
                                           "_controller/ur_driver/URScript",
                                           std_msgs.msg.String, queue_size=1)
        self._rospack = rospkg.RosPack()
        self._read_templates()

    # Publish various programs
    def wait(timeout_duration=rospy.Duration.from_sec(20.0)):
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

    def lin_move(target_pose, acceleration=0.5, velocity=0.03):
        rospy.logdebug("UR script lin move uses the ee_link of the robot, not the EE of the move group.")
        rospy.logdebug("original pose:")
        rospy.logdebug(target_pose)

        transform_success = False
        counter = 50
        while not transform_success:
            try:
                counter += 1
                robot_pose = self._listener.transformPose(
                                self._robot_name + "_base", target_pose)
                transform_success = True
            except:
                rospy.logdebug("Failed to transform from frame "
                               + target_pose.header.frame_id
                               + ". Waiting for .1 seconds")
                rospy.sleep(.1)

        xyz = [robot_pose.pose.position.x,
               robot_pose.pose.position.y, robot_pose.pose.position.z]
        q   = [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y,
               robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]
        rpy = tf.transformations.euler_from_quaternion(q)
        # rpy needs to be in axis-angle representation
        # http://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/python-code-example-of-converting-rpyeuler-angles-to-rotation-vectorangle-axis-for-universal-robots/
        rospy.logdebug("q in robot base:")
        rospy.logdebug(q)

        # This seems to work, but it uses the ee_link TCP of the robot.
        program  = ""
        program += "def move_to_pose_lin():\n"
        program += "    textmsg(\"Move_l to a pose.\")\n"
        program += "    rv = rpy2rotvec([" \
                 + str(rpy[0]) + "," \
                 + str(rpy[1]) + "," \
                 + str(rpy[2]) + "])\n"
        program += "    target_pos=p[" \
                 + str(xyz[0]) + "," \
                 + str(xyz[1]) + "," \
                 + str(xyz[2]) + ", rv[0], rv[1], rv[2]]\n"
        program += "    movel(pose_trans(p[0.0,0.0,0.0,0.0,0.0,0.0], target_pos), a = " \
                 + str(acceleration) + ", v = " \
                 + str(velocity) + ")\n"
        program += "    textmsg(\"Done.\")\n"
        program += "end\n"
        return _publish_program(program)

    def lin_mov_rel(self, translation, acceleration=0.5, velocity=0.03):
        xyz = [translation.x, translation.y, translation.z]
        program = ""
        program += "def move_lin_rel():\n"
        program += "    textmsg(\"Move_l via relative translation.\")\n"
        program += "    current_pos = get_actual_tcp_pose()\n"
        program += "    offset_pose = p[" \
                 + str(xyz[0]) + ", " \
                 + str(xyz[1]) + ", " \
                 + str(xyz[2]) + ", 0.0, 0.0, 0.0]\n"
        program += "    movel(pose_trans(current_pos, offset_pose), a = " \
                 + str(acceleration) + ", v = " \
                 + str(velocity) + ")\n"
        program += "    textmsg(\"Done.\")\n"
        program += "end\n"
        return _publish_program(program)

    def horizontal_insertion(self, max_force=10.0, force_direction="Y-",
                             forward_speed=0.02, max_approach_distance=0.1,
                             max_radius=0.007, radius_increment=0.0003,
                             max_insertion_distance=0.035, impedance_mass=10,
                             peck_mode=False):
        ### Function definitions, for reference:
        ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
        ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):
        program_front = self.horizontal_insertion_template
        program_back  = ""
        program_back += "    rq_zero_sensor()\n"
        program_back += "    textmsg(\"Approaching.\")\n"
        program_back += "    rq_linear_search(\"" \
                      + force_direction + "\"," \
                      + str(max_force) + "," \
                      + str(forward_speed) + "," \
                      + str(max_approach_distance) + ")\n"
        program_back += "    max_insertion_distance = " \
                      + str(max_insertion_distance) + "\n"
        program_back += "    textmsg(\"Spiral searching.\")\n"
        program_back += "    sleep(3.0)\n"
        program_back += "    if rq_spiral_search_new(max_insertion_distance," \
                      + str(max_force) + ", " \
                      + str(max_radius*1000) + ", " \
                      + str(radius_increment*1000) + ", peck_mode=" \
                      + str(peck_mode) + "):\n"
        program_back += "        #Insert the Part into the bore#\n"
        program_back += "        textmsg(\"Impedance insert\")\n"
        program_back += "        sleep(3.0)\n"
        program_back += "        rq_impedance(max_insertion_distance, " \
                      + str(impedance_mass) + ")\n"
        program_back += "    end\n"
        program_back += "    textmsg(\"Done. Exiting.\")\n"
        program_back += "end\n"
        return _publish_program(program_front + "\n" + program_back)

    def insertion(self, max_force=10.0, force_direction="Z+",
                  forward_speed=0.02, max_approach_distance=0.1,
                  max_radius=0.004, radius_increment=0.0003,
                  max_insertion_distance=0.035, impedance_mass=10,
                  peck_mode=False):
        ### Function definitions, for reference:
        ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
        ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):
        program_front = self.insertion_template
        program_back  = ""
        program_back += "    rq_zero_sensor()\n"
        program_back += "    textmsg(\"Approaching.\")\n"
        program_back += "    rq_linear_search(\"" \
                      + force_direction + "\"," \
                      + str(max_force) + "," \
                      + str(forward_speed) + "," \
                      + str(max_approach_distance) + ")\n"
        program_back += "    max_insertion_distance = " \
                      + str(max_insertion_distance) + "\n"
        program_back += "    textmsg(\"Spiral searching.\")\n"
        program_back += "    sleep(3.0)\n"
        program_back += "    if rq_spiral_search_new(max_insertion_distance," \
                      + str(max_force) + ", " \
                      + str(max_radius*1000) + ", " \
                      + str(radius_increment*1000) + ", peck_mode=" \
                      + str(peck_mode) + "):\n"
        program_back += "        #Insert the Part into the bore#\n"
        program_back += "        textmsg(\"Impedance insert\")\n"
        program_back += "        sleep(3.0)\n"
        program_back += "        rq_impedance(max_insertion_distance, " \
                      + str(impedance_mass) + ")\n"
        program_back += "    end\n"
        program_back += "    textmsg(\"Done. Exiting.\")\n"
        program_back += "end\n"
        return _publish_program(program_front + "\n" + program_back)

    def linear_push(self, max_force=10.0, force_direction="Z+",
                    max_approach_distance=0.1, forward_speed=0.02):
        ### Function definitions, for reference:
        ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
        program_front = self.linear_push_template
        program_back  = ""
        program_back += "    rq_zero_sensor()\n"
        program_back += "    textmsg(\"Approaching linearly.\")\n"
        program_back += "    rq_linear_search(\"" + force_direction + "\"," \
                      + str(max_force) + "," \
                      + str(forward_speed) + "," \
                      + str(max_approach_distance) + ")\n"
        program_back += "    textmsg(\"Done.\")\n"
        program_back += "end\n"
        return _publish_program(program_front + "\n" + program_back)

    def spiral_motion(self, acceleration=0.1, velocity=0.03,
                      max_radius=0.0065, radius_increment=0.002,
                      theta_increment=30, spiral_axis="Z"):
        if (radius_increment < 0.0001) or (radius_increment > 0.005):
            rospy.logerr("radius_incr needs to be between 0.0001 and 0.005 but is " + str(radius_increment))
            return False

        program_front = self.spiral_motion_template
        program_back = ""
        program_back += "    textmsg(\"Performing spiral motion.\")\n"
        program_back += "    spiral_motion(" \
                      + str(max_radius) + ", " \
                      + str(radius_increment) + ", " \
                      + str(velocity) + ", " \
                      + str(acceleration) + ", \"" \
                      + spiral_axis + "\", " \
                      + str(theta_increment) + ")\n"
        program_back += "    textmsg(\"Done.\")\n"
        program_back += "end\n"
        return _publish_program(program_front + "\n" + program_back)

    def move_j(self, joint_positions, acceleration=0.5, velocity=0.5):
        if not len(joint_positions) == 6:
            rospy.logwarn("Joint pose vector not of the correct length")
            return False

        program  = ""
        program += "def move_to_joint_pose():\n"
        program += "    textmsg(\"Move_j to a pose.\")\n"
        program += "    target_pos=[" + str(joint_positions[0]) + "," \
                 + str(joint_positions[1]) + "," \
                 + str(joint_positions[2]) + "," \
                 + str(joint_positions[3]) + "," \
                 + str(joint_positions[4]) + "," \
                 + str(joint_positions[5]) + "]\n"
        program += "    movej(target_pos, a = " \
                 + str(acceleration) + ", v = " \
                 + str(velocity) + ")\n"
        program += "    textmsg(\"Done.\")\n"
        program += "end\n"
        return _publish_program(program)

    # Private functions
    def _publish_program(self, program):
        rospy.loginfo("Sending UR robot program ")
        # rospy.logdebug("Program is:")
        # rospy.logdebug(program)
        program_msg = std_msgs.msg.String()
        program_msg.data = program
        self._publisher.publish(program_msg)
        return True

    def _read_templates(self):
        # Read the files containing the program templates into memory
        self.insertion_template \
            = self._read_template("peginholespiral_imp_osx.script")
        self.horizontal_insertion_template \
            = self._read_template("peginholespiral_imp_osx_y_negative.script")
        self.linear_push_template \
            = self._read_template("linear_search_short.script")
        self.spiral_motion_template \
            = self._read_template("spiral_motion.script")
        return True

    def _read_template(self, filename):
        program_template_file = open(os.path.join(self._rospack.get_path(
                                                    "o2as_skills"),
                                                  "src/urscript", filename),
                                     'rb')
        program_line = program_template_file.read(1024)
        linecounter = 0
        template = ""
        while program_line:
            template += program_line
            program_line = program_template_file.read(1024)
        return template

    def _is_program_running(self):
        msg = rospy.wait_for_message("/" + robot_name +
                                     "_controller/ur_driver/robot_mode_state",
                                     ur_msgs.RobotModeDataMsg)
        if msg:
            return msg.is_program_running
        else:
            rospy.logerr("No message received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
            return False
