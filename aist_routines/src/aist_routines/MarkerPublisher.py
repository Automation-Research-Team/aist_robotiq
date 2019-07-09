import rospy
import copy
import collections

from math import pi
from tf   import transformations as tfs

from geometry_msgs      import msg as gmsg
from visualization_msgs import msg as vmsg
from std_msgs           import msg as smsg

######################################################################
#  class MarkerPublisher                                             #
######################################################################
class MarkerPublisher(object):
    MarkerProps = collections.namedtuple("MarkerProps",
                                         "draw_axes, scale, color")
    _marker_props = {
        "pose" :
            MarkerProps(True,  (0.006, 0.015, 0.015), (0.0, 1.0, 0.0, 0.8)),
        "pick_pose":
            MarkerProps(True,  (0.006, 0.015, 0.015), (1.0, 0.0, 1.0, 0.8)),
        "place_pose":
            MarkerProps(True,  (0.006, 0.015, 0.015), (0.0, 1.0, 1.0, 0.8)),
        "graspability":
            MarkerProps(True,  (0.004, 0.004, 0.004), (1.0, 1.0, 0.0, 0.8)),
        "":
            MarkerProps(False, (0.004, 0.004, 0.004), (0.0, 1.0, 0.0, 0.8)),
        }

    def __init__(self):
        super(MarkerPublisher, self).__init__()
        self._pub = rospy.Publisher("visualization_marker",
                                    vmsg.MarkerArray, queue_size=10)
        self._nmarkers = 0

    def delete_all(self):
        markers       = vmsg.MarkerArray()
        marker        = vmsg.Marker()
        marker.ns     = "markers"
        marker.action = vmsg.Marker.DELETEALL
        markers.markers.append(marker)
        self._pub.publish(markers)
        self._nmarkers = 0

    def add(self, marker_pose, marker_type, text="", lifetime=15):
        marker_prop = MarkerPublisher._marker_props[marker_type]

        markers             = vmsg.MarkerArray()
        marker              = vmsg.Marker()
        marker.header       = marker_pose.header
        marker.header.stamp = rospy.Time.now()
        marker.ns           = "markers"
        marker.action       = vmsg.Marker.ADD
        marker.lifetime     = rospy.Duration(lifetime)

        if marker_prop.draw_axes:  # Draw frame axes?
            smax = max(*marker_prop.scale)
            smin = min(*marker_prop.scale)

            marker.type  = vmsg.Marker.ARROW
            marker.pose  = marker_pose.pose
            marker.scale = gmsg.Vector3(2.5*smax, 0.5*smin, 0.5*smin)
            marker.color = smsg.ColorRGBA(1.0, 0.0, 0.0, 0.8)  # red
            marker.id    = self._nmarkers
            self._nmarkers += 1
            markers.markers.append(copy.deepcopy(marker))  # x-axis

            marker.pose  = self._pose_rotated_by_rpy(marker_pose.pose,
                                                     0, 0, pi/2)
            marker.color = smsg.ColorRGBA(0.0, 1.0, 0.0, 0.8)  # green
            marker.id    = self._nmarkers
            self._nmarkers += 1
            markers.markers.append(copy.deepcopy(marker))  # y-axis

            marker.pose  = self._pose_rotated_by_rpy(marker_pose.pose,
                                                     0, -pi/2, 0)
            marker.color = smsg.ColorRGBA(0.0, 0.0, 1.0, 0.8)  # blue
            marker.id    = self._nmarkers
            self._nmarkers += 1
            markers.markers.append(copy.deepcopy(marker))  # z-axis

        marker.type  = vmsg.Marker.SPHERE
        marker.pose  = marker_pose.pose
        marker.scale = gmsg.Vector3(*marker_prop.scale)
        marker.color = smsg.ColorRGBA(*marker_prop.color)
        marker.id    = self._nmarkers
        self._nmarkers += 1
        markers.markers.append(copy.deepcopy(marker))

        if text != "":
            marker.pose.position.z -= (marker.scale.z + 0.001)
            marker.type  = vmsg.Marker.TEXT_VIEW_FACING
            marker.color = smsg.ColorRGBA(1.0, 1.0, 1.0, 0.8)  # white
            marker.text  = text
            marker.id    = self._nmarkers
            self._nmarkers += 1
            markers.markers.append(copy.deepcopy(marker))

        self._pub.publish(markers)

    def _pose_rotated_by_rpy(self, pose, roll, pitch, yaw):
        pose_rotated = copy.deepcopy(pose)
        pose_rotated.orientation = gmsg.Quaternion(
            *tfs.quaternion_multiply((pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w),
                                     tfs.quaternion_from_euler(roll,
                                                               pitch, yaw)))
        return pose_rotated
