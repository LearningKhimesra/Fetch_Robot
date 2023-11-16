#!/usr/bin/env python

import rospy
import math
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class RobotMover:

    def __init__(self):
        # MoveIt! Initialization
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")  
        self.base_group = MoveGroupCommander("base")  
        
        self.arm_group.set_planning_time(10)  # Increase planning time
        self.pose_sub = rospy.Subscriber('/detected_object_pose', PoseStamped, self.pose_callback)
        
        self.initial_object_pose = None  # To remember the initial pose of the object

    def approach_object(self, x, y, z):
        target_pose = self.arm_group.get_current_pose().pose
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z + 0.1 

        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()
        success = self.arm_group.execute(plan, wait=True)

        if not success:
            rospy.logwarn("Execution failed! Could not reach the target pose.")
            return False

        return True

    def grasp_object(self):
        gripper_target = 0.01
        self.gripper_group.set_joint_value_target([gripper_target])
        self.gripper_group.go(wait=True)

    def release_object(self):
        gripper_target = 0.1
        self.gripper_group.set_joint_value_target([gripper_target])
        self.gripper_group.go(wait=True)

    def rotate_base(self, angle):
        
        current_pose = self.base_group.get_current_pose().pose
        current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        
        # Calculate the desired orientation
        r, p, y = euler_from_quaternion(current_orientation)
        y += math.radians(angle)  # Convert angle to radians and add to current yaw
        q = quaternion_from_euler(r, p, y)

        # Set the new orientation
        target_pose = PoseStamped()
        target_pose.pose.orientation = Quaternion(*q)
        self.base_group.set_pose_target(target_pose)
        self.base_group.go(wait=True)

    def pose_callback(self, data):
        if not self.initial_object_pose:
            self.initial_object_pose = data.pose.position

        if self.approach_object(data.pose.position.x, data.pose.position.y, data.pose.position.z):
            self.grasp_object()
            
            # Rotate the robot base by 90 degrees
            self.rotate_base(90)

            # Approach the initial position of the object
            self.approach_object(self.initial_object_pose.x, self.initial_object_pose.y, self.initial_object_pose.z)

            # Release the object
            self.release_object()

if __name__ == "__main__":
    rospy.init_node('fetch_robot_mover', anonymous=True)
    rm = RobotMover()
    rospy.spin()

