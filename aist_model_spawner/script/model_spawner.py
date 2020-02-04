#!/usr/bin/env python

import os, re
import rospy, rospkg, tf
import xacro
import xml.dom.minidom
from tf                 import transformations as tfs
from aist_model_spawner import srv as msrv
from aist_model_spawner import msg as mmsg

#########################################################################
#  class ModelSapwnerServer                                             #
#########################################################################
class ModelSpawnerServer(object):

    _BeginRobot = "<robot name=\"model_description\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n  <link name=\"{}\"/>\n"
    _Macro      = "  <xacro:include filename=\"{1}\"/>\n  <xacro:{0} prefix=\"{2}\" parent=\"{3}\" spawn_attached=\"true\">\n    <origin xyz=\"{4} {5} {6}\" rpy=\"{7} {8} {9}\"/>\n  </xacro:{0}>\n"
    _EndRobot   = "</robot>\n"

    def __init__(self, urdf_dir):
        super(ModelSpawnerServer, self).__init__()

        self._add_srv        = rospy.Service("~add", msrv.Add, self._add_cb)
        self._delete_srv     = rospy.Service("~delete", msrv.Delete,
                                             self._delete_cb)
        self._delete_all_srv = rospy.Service("~delete_all", msrv.DeleteAll,
                                             self._delete_all_cb)
        self._get_list_srv   = rospy.Service("~get_list", msrv.GetList,
                                             self._get_list_cb)
        self._urdf_dir       = urdf_dir
        self._parent_frame   = ""
        self._macros         = {}
        self._robot          = None
        self._broadcaster    = tf.TransformBroadcaster()
        self._publisher      = rospy.Publisher("~model_description",
                                               mmsg.ModelDescription,
                                               queue_size=10)
        self._update_robot()

    def tick(self):
        now = rospy.Time.now()
        for childNode in self._robot.childNodes:
            if childNode.localName != "joint" or \
               childNode.getAttribute("type") != "fixed":
                continue

            try:
                element = childNode.getElementsByTagName("parent")[0]
                parent  = element.getAttribute("link")
                element = childNode.getElementsByTagName("child")[0]
                child   = element.getAttribute("link")
                element = childNode.getElementsByTagName("origin")[0]
                xyz     = map(float, element.getAttribute("xyz").split())
                rpy     = map(float, element.getAttribute("rpy").split())

                self._broadcaster.sendTransform(
                    xyz, tfs.quaternion_from_euler(*rpy), now, child, parent)
            except Exception as e:
                rospy.logwarn(e)

    def _add_cb(self, req):
        parent_frame = req.pose.header.frame_id

        if self._parent_frame == "":
            self._parent_frame = parent_frame
        elif self._parent_frame != parent_frame:
            lospy.logerr("Requested parent frame[" + parent_frame +
                         "] is differnt from current parent frame[" +
                         self._frame + "].")
            return msrv.AddResponse(False)

        name        = req.name
        position    = req.pose.pose.position
        orientation = req.pose.pose.orientation
        macro_name  = "assy_part_" + re.split("[_-]", name)[0]

        self._macros[name] = ModelSpawnerServer._Macro.format(
                                macro_name,
                                os.path.join(self._urdf_dir,
                                             name + "_macro.urdf.xacro"),
                                "", parent_frame,
                                position.x, position.y, position.z,
                                *tfs.euler_from_quaternion([orientation.x,
                                                            orientation.y,
                                                            orientation.z,
                                                            orientation.w]))

        success = self._update_robot()
        if not success:
            self._delete_macro(name)
        return msrv.AddResponse(success)

    def _delete_cb(self, req):
        self._delete_macro(req.name)
        return msrv.DeleteResponse(self._update_robot())

    def _delete_all_cb(self, req):
        self._macros.clear()
        self._parent_frame = ""
        return msrv.DeleteAllResponse(self._update_robot())

    def _get_list_cb(self, req):
        return msrv.GetListResponse(self._macros.keys())

    def _delete_macro(name):
        try:
            del self._macros[name]
            if not self._macros:
                self._parent_frame = ""
        except KeyErr:
            rospy.logerr("Tried to delete unknown model[" + name + "].")

    def _update_robot(self):
        try:
            desc = self._create_description()
            self._publisher.publish(desc)
            self._robot = xml.dom.minidom.parseString(desc).childNodes[0]
            return True
        except Exception as e:
            rospy.logerr(e)
            return False

    def _create_description(self):
        # Create Model description in Xacro format.
        xacro_desc = ModelSpawnerServer._BeginRobot.format(self._parent_frame)
        for macro in self._macros.values():
            xacro_desc += macro
        xacro_desc += ModelSpawnerServer._EndRobot

        # Expand and process Xacro into XML format.
        doc = xacro.parse(xacro_desc)  # Create DOM tree.
        xacro.process_doc(doc)         # Expand and process macros.
        return doc.toprettyxml(indent='  ', encoding='utf8')


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node("aist_model_spwner")

    rospack    = rospkg.RosPack()
    urdf_dir   = rospy.get_param(
                     "urdf_dir",
                     os.path.join(rospack.get_path("o2as_parts_description"),
                                  "urdf/generated"))

    spawner = ModelSpawnerServer(urdf_dir)
    rate    = rospy.Rate(1)
    while not rospy.is_shutdown():
        spawner.tick()
        rate.sleep()
