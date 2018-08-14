#!/usr/bin/env python

import cv2
import numpy as np
import math
from o2as_circle_detection.srv import *
import rospy

class ObjectDetection:
    __distance_constant = 0.326
    __distance_constant_cam2 = 0.259
    __object_position = {"cp": (288, 346),
                         "ps": (298, 356),
                         "pss": (255, 399)}

    def calculate_distance(self, height, width, obj, camera_constant):
        distanceHeight = (height - self.__object_position[obj][1])
        distanceWidth = (width - self.__object_position[obj][0])

        # print(distanceHeight)
        if(camera_constant == 2):
            return (distanceHeight * self.__distance_constant, distanceWidth * self.__distance_constant)

        else:
            return (distanceHeight * self.__distance_constant_cam2, distanceWidth * self.__distance_constant_cam2)


    def capture_image(self, camera_number):
        cap = cv2.VideoCapture(camera_number)
        ret, frame = cap.read()
        Height, Width = frame.shape[:2]
        img = cv2.resize(frame, (int(Width), int(Height)))
        output = img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 5);
        gray = cv2.medianBlur(gray, 7)

        # Adaptive Guassian Threshold is to detect sharp edges in the Image. For more information Google it.
        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
                                     cv2.THRESH_BINARY, 11, 3.5)

        kernel = np.ones((5, 5), np.uint8)
        gray = cv2.erode(gray, kernel, iterations=1)
        # gray = erosion

        gray = cv2.dilate(gray, kernel, iterations=1)
        # gray = dilation

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.25, 5, param1=5, param2=24, minRadius=18,
                                   maxRadius=35)
        # # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")

            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                print("Height: " + str(Height - y))
                print("Width: " + str(Width - x))
                # print(self.calculate_distance(Height - y, Width - x))

                cv2.imshow("object_detected", output)
                if cv2.waitKey() & 0xFF == ord('q'):
                    break
        #cv.imshow("joe_my_man",img)


    def capture_until_one_circle_detected(self, camera, obj):
        cap = cv2.VideoCapture(camera)
        font = cv2.FONT_HERSHEY_SIMPLEX
        circle_not_detected=True
        while circle_not_detected:
            # Capture frame-by-frame
            ret, frame = cap.read()
            Height, Width = frame.shape[:2]
            img = cv2.resize(frame, (int(Width), int(Height)))

            # load the image, clone it for output, and then convert it to grayscale

            output = img.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # apply GuassianBlur to reduce noise. medianBlur is also added for smoothening, reducing noise.
            gray = cv2.GaussianBlur(gray, (5, 5), 0);
            gray = cv2.medianBlur(gray, 5)

            # Adaptive Guassian Threshold is to detect sharp edges in the Image. For more information Google it.
            gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
                                         cv2.THRESH_BINARY, 11, 3.5)

            kernel = np.ones((5, 5), np.uint8)
            gray = cv2.erode(gray, kernel, iterations=1)
            # gray = erosion

            gray = cv2.dilate(gray, kernel, iterations=1)
            # gray = dilation

            # detect circles in the image
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.25, 5, param1=5, param2=24, minRadius=18,
                                       maxRadius=35)
            print circles
            
            # ensure at least some circles were found
            if circles is not None:
                if int(circles.shape[1]) == 1:
                    # convert the (x, y) coordinates and radius of the circles to integers
                    circles = np.round(circles[0, :]).astype("int")
                    # loop over the (x, y) coordinates and radius of the circles
                    for (x, y, r) in circles:
                        # draw the circle in the output image, then draw a rectangle in the image
                        # corresponding to the center of the circle

                        distance = self.calculate_distance(Height - y, Width - x, obj, camera)
                        #if abs(distance[0]) > 60 or abs(distance[1]) > 20:
                        #    continue
                        if False:
                            print "yo"
                        else:
                            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                            # cv2.putText(output, str(distance[0]), (x - 250, y), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                            # cv2.putText(output, str(distance[1]), (x - 100, y), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        cv2.imwrite('output.png',output)
                        circle_not_detected=False
                        
                        return (-distance[0], -distance[1])
            # cv2.imshow('output', output)
            print "lala2"
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

    def my_callback(self, req):
        rospy.loginfo("Circle detection callback has been called")
        res = CircleDetectionCommandResponse()
        distance=self.capture_until_one_circle_detected(req.camera_id,req.object_name)
        #rospy.logerr('No command sent to the gripper, service request was empty.')
        res.x_distance=distance[0]
        res.y_distance=distance[1]
        res.circle_detected = True
        return res

        #res.circle_detected = True
        #return res

if __name__ == "__main__":
    rospy.init_node("circle_detection_server")
    object_detection = ObjectDetection()

    #object_detection.realtime_capture(0,"cp")
    #object_detection.capture_image(0)

    my_service = rospy.Service('circle_detection_command', CircleDetectionCommand, object_detection.my_callback)
    rospy.loginfo("Service circle_detection is ready")
    rospy.spin()
