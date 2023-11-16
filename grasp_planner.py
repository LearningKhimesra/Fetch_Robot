#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String

class GraspPlanner:

    def __init__(self):
        rospy.init_node('fetch_grasp_planner', anonymous=True)

        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm_with_torso")

       
        self.gripper_pub = rospy.Publisher('/gripper_controller/command', String, queue_size=10)

       
        self.object_pose_sub = rospy.Subscriber('/object_pose', geometry_msgs.msg.PoseStamped, self.object_pose_callback)

    def object_pose_callback(self, pose_msg):
        # Plan to pre-grasp pose
        pre_grasp_pose = self.calculate_pre_grasp_pose(pose_msg)
        self.group.set_pose_target(pre_grasp_pose)
        plan = self.group.plan()
        self.group.execute(plan, wait=True)

       
        self.gripper_pub.publish("open")

        
        self.group.set_pose_target(pose_msg)
        plan = self.group.plan()
        self.group.execute(plan, wait=True)

       
        self.gripper_pub.publish("close")

        # Optionally: Lift object
        lift_pose = pose_msg
        lift_pose.pose.position.z += 0.1  # Lift 10 cm
        self.group.set_pose_target(lift_pose)
        plan = self.group.plan()
        self.group.execute(plan, wait=True)

    def calculate_pre_grasp_pose(self, grasp_pose):
        pre_grasp_pose = grasp_pose
        pre_grasp_pose.pose.position.z += 0.1  # Offset by 10 cm in z-direction
        return pre_grasp_pose

if __name__ == '__main__':
    grasp_planner = GraspPlanner()
    rospy.spin()

