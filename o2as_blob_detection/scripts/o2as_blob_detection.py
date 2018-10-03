#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Float64
import geometry_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import cv2
import sensor_msgs.point_cloud2 as pc2

from PIL import Image as PIL_Image
from PIL import ImageDraw as PIL_ImageDraw
from geometry_msgs.msg import Polygon, Point32 

import copy


class BlobDetection(object):
    def __init__(self):
        #Variable
        self.bridge = CvBridge()
        current_image = Image()
        current_cloud = PointCloud2()

        # Config parameters
        # TODO: read values from config file
        self.image_topic = "/camera/color/image_raw"
        self.cloud_topic = "/camera/cloud"
        self.blob_pos_img_topic = rospy.get_name()+"/blob_pos_img"
        self.blob_pos_cloud_topic = rospy.get_name()+"/blob_pos_cloud"

        self.pub_img_pos = rospy.Publisher(self.blob_pos_img_topic, geometry_msgs.msg.Point, queue_size=10)
        self.pub_cloud_pos = rospy.Publisher(self.blob_pos_cloud_topic, geometry_msgs.msg.Point, queue_size=10)

        # Subscriber
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback)



    # Callback
    def image_callback(self, msg_in):
      # Convert the image to be used wit OpenCV library
      img_cv = self.bridge.imgmsg_to_cv2(msg_in, desired_encoding="passthrough")
      img = np.asarray(img_cv)[:, :, ::-1]

      # Apply the mask
      mask_u = 200
      mask_v = 100
      #test_polygon = [ (mask_u,mask_v),
      #                 (msg_in.width-mask_u,mask_v),
      #                 (msg_in.width-mask_u,msg_in.height-mask_v),
      #                 (mask_u,msg_in.height-mask_v)]

      test_polygon = Polygon()
      test_polygon.points = [Point32(mask_u,mask_v,0),
                             Point32(msg_in.width-mask_u,mask_v,0),
                             Point32(msg_in.width-mask_u,msg_in.height-mask_v,0),
                             Point32(mask_u,msg_in.height-mask_v,0)]

      #test_polygon = geometry_msgs.msg.Point()
      #test_polygon [0] = geometry_msgs_msg.Point(mask_u,mask_v,0) 
      #test_polygon [1] = geometry_msgs_msg.Point(msg_in.width-mask_u,mask_v)
      #test_polygon [2] = geometry_msgs_msg.Point(msg_in.width-mask_u,msg_in.height-mask_v)
      #test_polygon [3] = geometry_msgs_msg.Point(mask_u,msg_in.height-mask_v)

      masked_img= self.mask_image(img, test_polygon)

      # Detect the blob in the image
      res_b, blob_array = self.detect_blob(masked_img)
      print(blob_array)
   
      if(res_b):
          msg_out = geometry_msgs.msg.Point()
          msg_out = blob_array[0]
          self.pub_img_pos.publish(msg_out)

          # Convert the blob position image in the 3D camera system
          msg_out = self.compute_3D_pos1(self.current_cloud, int(blob_array[0].x), int(blob_array[0].y))

          # Publish the results   
          self.pub_cloud_pos.publish(msg_out)

          # Slow down this node and reduce data transfer for image
          #rospy.sleep(0.01)
      else:
          pass

          # Convert the blob position image in the 3D camera system
          # Publish the results   

          # Slow down this node and reduce data transfer for image
          #rospy.sleep(0.01)


    def cloud_callback(self, msg_in):
      self.current_cloud = copy.deepcopy(msg_in)

    def compute_3D_pos1(self, in_cloud, u,v):
      # TODO the projection require int but the precision of the blob detected are in float so there is a loss of accuracy 
    
      data_out = pc2.read_points(in_cloud, field_names = None, skip_nans=False, uvs=[[u, v]]) 
      int_data = next(data_out)

      res_out = geometry_msgs.msg.Point()
      res_out.x = int_data[0]  
      res_out.y = int_data[1]
      res_out.z = int_data[2] 

      return res_out

    def mask_image(self, in_img_cv, in_polygon):

        polygon = [ (in_polygon.points[0].x,in_polygon.points[0].y),
                    (in_polygon.points[1].x,in_polygon.points[1].y),
                    (in_polygon.points[2].x,in_polygon.points[2].y),
                    (in_polygon.points[3].x,in_polygon.points[3].y)] 
 
        #print(in_img_cv.shape)
        # Create the mask from the polygon
        mask_img = PIL_Image.new('L', (in_img_cv.shape[1],in_img_cv.shape[0]), 0)
        # Draw the mask with the polygon coordinate
        PIL_ImageDraw.Draw(mask_img).polygon(polygon, outline = 1, fill = 255)
        mask_image_np = np.array(mask_img)

        #DEBUG Ssave the mask in a file
        #namefile = "mask.png"
        #mask_img.save(namefile,'PNG')

        # Apply the mask to the image
        #img_cv = cv2.imread(img_to_mask)
        #mask_cv = cv2.imread(mask_img,0)
        img_cv = in_img_cv
        out_img = cv2.bitwise_and(img_cv,img_cv, mask = mask_image_np)
        #cv2.imwrite('masked_image.png',out_img)
        return out_img

    def detect_blob(self, img):
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
    
        blob_array = []
     
        if(len(keypoints)):
            for i in range(len(keypoints)):
                blob= geometry_msgs.msg.Point()
                blob.x = keypoints[i].pt[0]
                blob.y = keypoints[i].pt[1]
                blob_array.append(blob)
            return True, blob_array

        else:  

            return False, blob_array
 

if __name__ == "__main__":


  # Initialization
  rospy.init_node("o2as_blob_detection")
  node = BlobDetection()

  rospy.spin()
