#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Float64
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def detect_blob(img):
    """Compute the ratio of red area in the image.
  
    The returned value should be used to check if the precision gripper pick a
    part or not. Assumption here is that the picked part will decrease the
    ratio of red area; low value implies the gripper picks a part.
  
    Return True if bg_ratio < bg_threshold, False otherwise.
  
    :param img: RGB image, 8bit (0-255), numpy.ndarray, shape=(w, h, 3)
    :param int x: x-axis of Upper left of ROI.
    :param int y: y-axis of Upper left of ROI.
    :param int w: Width of ROI.
    :param int h: Height of ROI.
    """
    im_rgb = img
    im_gray = cv2.cvtColor(im_rgb,cv2.COLOR_BGR2GRAY)

    params = cv2.SimpleBlobDetector_Params()

    # Segmentation Thresholds
    params.minThreshold = 100
    params.maxThreshold = 400

    # Filter by color
    params.filterByColor = True
    params.blobColor = 0

    # Filter by size of the blob.
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 150

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.7

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

   # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(im_gray)

    # Draw the key points 
    im_with_keypoints = cv2.drawKeypoints(im_rgb, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show blobs
    cv2.imwrite("blob_detection_results.png", im_with_keypoints)

    blob = geometry_msgs.msg.Point()
    if(len(keypoints)):
        blob.x = keypoints[0].pt[0]
        blob.y = keypoints[0].pt[1]
    else:
        blob.x = -666
        blob.y = -666

    return blob

if __name__ == "__main__":
  # Config parameters
  # TODO: read values from config file
  node_name = "o2as_blob_detection"
  image_topic = "/camera/color/image_raw"
  blob_position_topic = node_name+"/blob_position"

  # Initialization
  rospy.init_node(node_name)
  pub = rospy.Publisher(blob_position_topic, geometry_msgs.msg.Point, queue_size=10)
  bridge = CvBridge()

  # Callback
  def callback(msg_in):
    img = bridge.imgmsg_to_cv2(msg_in, desired_encoding="passthrough")
    img = np.asarray(img)[:, :, ::-1]
    blob_point = detect_blob(img)
    msg_out = geometry_msgs.msg.Point()
    msg_out = blob_point
    pub.publish(msg_out)

    # Slow down this node and reduce data transfer for image
    rospy.sleep(0.01)

  # Subscriber
  rospy.Subscriber(image_topic, Image, callback)

  rospy.spin()
