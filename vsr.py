#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class VisualServoing:

    def __init__(self):
        rospy.init_node('fetch_visual_servoing', anonymous=True)

        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.image_callback)

        
        self.velocity_pub = rospy.Publisher('/base_controller/command', geometry_msgs.msg.Twist, queue_size=10)

        
        self.target_position = np.array([320, 240])  # Example values, adjust according to your setup

       
        self.Kp = 0.001  # Proportional gain

    def image_callback(self, msg):
        # Convert image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        
        object_position = np.array([100, 200])  
        # Calculate position error
        error = self.target_position - object_position

        # Apply control law
        velocity_command = geometry_msgs.msg.Twist()
        velocity_command.linear.x = self.Kp * error[0]
        velocity_command.linear.y = self.Kp * error[1]

       
        self.velocity_pub.publish(velocity_command)

if __name__ == '__main__':
    visual_servoing = VisualServoing()
    rospy.spin()

