#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped

class RobotMover:

    def __init__(self):
        # MoveIt! Initialization
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")  
        
        self.arm_group.set_planning_time(10) 
        self.pose_sub = rospy.Subscriber('/detected_object_pose', PoseStamped, self.pose_callback)

    def approach_object(self, x, y, z):
        target_pose = self.arm_group.get_current_pose().pose
        target_pose.position.x += float(x)
        target_pose.position.y += float(y)
        target_pose.position.z = float(z) + 0.1  # Added an offset to Z 

        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()
        success = self.arm_group.execute(plan, wait=True)

        if not success:
            rospy.logwarn("Execution failed! Could not reach the target pose.")
            return False

        return True

    def grasp_object(self):
        
        gripper_target = 0.01  # Close the gripper to 1cm
        self.gripper_group.set_joint_value_target([gripper_target])
        self.gripper_group.go(wait=True)

        # Lift the object slightly after grasping 
        target_pose = self.arm_group.get_current_pose().pose
        target_pose.position.z += 0.10
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True)

    def release_object(self):
        # Open the gripper to release the object
        gripper_target = 0.1  # Open the gripper to 10cm 
        self.gripper_group.set_joint_value_target([gripper_target])
        self.gripper_group.go(wait=True)

    def pose_callback(self, data):
        if self.approach_object(data.pose.position.x, data.pose.position.y, data.pose.position.z):
            self.grasp_object()
            
            self.release_object()

if __name__ == "__main__":
    rospy.init_node('fetch_robot_mover', anonymous=True)
    rm = RobotMover()
    rospy.spin()

