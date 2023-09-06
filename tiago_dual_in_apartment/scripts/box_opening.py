#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from giskardpy.python_interface import GiskardWrapper
from urdf_parser_py import urdf

import rospy

if __name__=="__main__":
    rospy.init_node("test_giskard")
    path_to_urdf = '/media/giangnguyen/Storage/Multiverse/multiverse_ws/src/Multiverse-Objects/articulated_objects/box/urdf/box.urdf'
    robot: urdf.Robot = urdf.Robot.from_xml_file(file_path=path_to_urdf)
    urdf = robot.to_xml_string()
    giskard = GiskardWrapper(node_name="giskard")
    giskard.clear_world()
    box_pose = PoseStamped()
    box_pose.header.frame_id = 'map'
    box_pose.pose.position.x = 2.5
    box_pose.pose.position.y = 2.5
    box_pose.pose.position.z = 1.05
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.707
    box_pose.pose.orientation.w = 0.707
    print(giskard.add_urdf(name='box',
                     urdf=urdf,
                     pose=box_pose,
                     parent_link='map'))
    print(urdf)
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

    # cart_goal = PoseStamped()
    # cart_goal.header.frame_id = "torso_lift_link"
    # cart_goal.pose.position.x = 0.5
    # giskard.set_cart_goal(goal_pose=cart_goal, tip_link="torso_lift_link", root_link="map")
    # giskard.plan_and_execute()