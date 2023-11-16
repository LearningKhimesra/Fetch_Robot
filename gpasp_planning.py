#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import cv2

class VisualServoing:

    def __init__(self):
        rospy.init_node('fetch_visual_servoing', anonymous=True)

        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.image_callback)

        
        self.velocity_pub = rospy.Publisher('/base_controller/command', geometry_msgs.msg.Twist, queue_size=10)


        self.grasp_trigger_pub = rospy.Publisher('/trigger_grasp', Bool, queue_size=10)

        # Target position in the camera frame
        self.target_position = np.array([320, 240]) 
        # Control gains
        self.Kp = 0.001  # Proportional gain

        self.error_threshold = 10 
    def image_callback(self, msg):
        # Convert image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

       
        
        object_position = np.array([100, 200])  
       
        error = self.target_position - object_position
        error_magnitude = np.linalg.norm(error)

        
        if error_magnitude < self.error_threshold:
            self.grasp_trigger_pub.publish(True)
        else:
           
            velocity_command = geometry_msgs.msg.Twist()
            velocity_command.linear.x = self.Kp * error[0]
            velocity_command.linear.y = self.Kp * error[1]

            
            self.velocity_pub.publish(velocity_command)

if __name__ == '__main__':
    visual_servoing = VisualServoing()
    rospy.spin()

