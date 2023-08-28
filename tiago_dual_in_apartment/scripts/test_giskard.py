#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from giskardpy.python_interface import GiskardWrapper

import rospy

if __name__=="__main__":
    rospy.init_node("test_giskard")

    giskard = GiskardWrapper(node_name="giskard")
    better_pose = {
        'arm_left_1_joint': - 1.0,
        'arm_left_2_joint': 0.0,
        'arm_left_3_joint': 1.5,
        'arm_left_4_joint': 2.2,
        'arm_left_5_joint': - 1.5,
        'arm_left_6_joint': 0.5,
        'arm_left_7_joint': 0.0,
        'arm_right_1_joint': - 1.0,
        'arm_right_2_joint': 0.0,
        'arm_right_3_joint': 1.5,
        'arm_right_4_joint': 2.2,
        'arm_right_5_joint': - 1.5,
        'arm_right_6_joint': 0.5,
        'arm_right_7_joint': 0.0,
        'torso_lift_joint': 0.35,
        'gripper_right_left_finger_joint': 0.045,
        'gripper_right_right_finger_joint': 0.045,
        'gripper_left_left_finger_joint': 0.045,
        'gripper_left_right_finger_joint': 0.045,
    }

    giskard.set_joint_goal(goal_state=better_pose)
    giskard.plan_and_execute()

    cart_goal = PoseStamped()
    cart_goal.header.frame_id = "torso_lift_link"
    cart_goal.pose.position.x = 0.5
    giskard.set_cart_goal(goal_pose=cart_goal, tip_link="torso_lift_link", root_link="map")
    giskard.plan_and_execute()