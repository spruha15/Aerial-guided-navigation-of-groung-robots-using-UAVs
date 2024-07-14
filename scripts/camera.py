#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraFeed:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris_downward_depth_camera/camera/rgb/image_raw", Image, self.image_callback)
        self.object_detector = cv2.createBackgroundSubtractorMOG2(history=100,varThreshold=40)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Apply background subtraction to create a mask
        mask = self.object_detector.apply(cv_image)
        self.contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in self.contours:

            self.area = cv2.contourArea(cnt)
            if self.area > 30 and self.area <200:

             #cv2.drawContours(cv_image,[cnt],-1,(0,255,0),2)
             self.x , self.y , self.w , self.h = cv2.boundingRect(cnt)
             cv2.rectangle(cv_image,(self.x,self.y),(self.x+self.w,self.y+self.h),(0,255,0),3)



        # Display the original image and the mask
        cv2.imshow("Drone Camera Feed", cv_image)
        cv2.imshow("Object Detection Mask", mask)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('drone_camera_feed', anonymous=True)
    camera_feed = CameraFeed()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
