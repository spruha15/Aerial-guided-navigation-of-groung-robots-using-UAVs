#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2

class CameraFeed:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris_downward_depth_camera/camera/rgb/image_raw", Image, self.image_callback)
    
    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Display the image
        cv2.imshow("Drone Camera Feed", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('drone_camera_feed', anonymous=True)
    camera_feed = CameraFeed()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
