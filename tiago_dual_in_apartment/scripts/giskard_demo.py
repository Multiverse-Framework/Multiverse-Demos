#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64

from giskardpy.python_interface import GiskardWrapper

import rospy

gripper_left_publishers = [
    rospy.Publisher("/gripper_left_left_finger_effort_controller/command", Float64, queue_size=10),
    rospy.Publisher("/gripper_left_right_finger_effort_controller/command", Float64, queue_size=10),
]

gripper_right_publishers = [
    rospy.Publisher("/gripper_right_left_finger_effort_controller/command", Float64, queue_size=10),
    rospy.Publisher("/gripper_right_right_finger_effort_controller/command", Float64, queue_size=10),
]


def control_gripper(open: bool, left: bool = True, right: bool = True) -> None:
    if open:
        rospy.loginfo("Open gripper")
    else:
        rospy.loginfo("Close gripper")

    if left:
        for gripper_left_publisher in gripper_left_publishers:
            gripper_left_publisher.publish(Float64(500) if open else Float64(-500))

    if right:
        for gripper_right_publisher in gripper_right_publishers:
            gripper_right_publisher.publish(Float64(500) if open else Float64(-500))

    if not open:
        rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("test_giskard")
    handle_name = "cabinet9_drawer2_handle"
    apartment_name = "apartment"
    left_gripper_name = "left_gripper_tool_frame"

    # initialize Giskard
    giskard = GiskardWrapper(node_name="giskard")
    giskard.clear_world()

    # open grippers
    control_gripper(open=True, left=True, right=True)

    # load apartment model into Giskard
    apartment_urdf = rospy.get_param("apartment_description")
    apartment_pose = PoseStamped()
    apartment_pose.header.frame_id = "map"
    apartment_pose.pose.orientation.w = 1
    giskard.add_urdf(name=apartment_name, urdf=apartment_urdf, pose=apartment_pose, parent_link="map")

    default_pose = {
        "arm_left_1_joint": -1.0,
        "arm_left_2_joint": 0.0,
        "arm_left_3_joint": 1.5,
        "arm_left_4_joint": 2.2,
        "arm_left_5_joint": -1.5,
        "arm_left_6_joint": 0.5,
        "arm_left_7_joint": 0.0,
        "arm_right_1_joint": -1.0,
        "arm_right_2_joint": 0.0,
        "arm_right_3_joint": 1.5,
        "arm_right_4_joint": 2.2,
        "arm_right_5_joint": -1.5,
        "arm_right_6_joint": 0.5,
        "arm_right_7_joint": 0.0,
        "torso_lift_joint": 0.35,
    }

    # move arms to default pose and drive in front of drawer
    giskard.set_joint_goal(goal_state=default_pose)
    cart_goal = PoseStamped()
    cart_goal.header.frame_id = "map"
    cart_goal.pose.position.x = 1.7
    cart_goal.pose.position.y = 2.5
    cart_goal.pose.orientation.w = 1  # identity rotation
    giskard.set_cart_goal(goal_pose=cart_goal, tip_link="base_footprint", root_link="map")
    giskard.plan_and_execute()

    # grasp handle
    # pregrasp pose
    left_gripper_goal = PoseStamped()
    left_gripper_goal.header.frame_id = handle_name
    left_gripper_goal.pose.position.x = -0.1
    left_gripper_goal.pose.orientation.x = 1  # 180-degree rotation about x-axis of gripper
    giskard.set_cart_goal(goal_pose=left_gripper_goal, tip_link=left_gripper_name, root_link="base_footprint")
    giskard.plan_and_execute()

    # grasp pose
    left_gripper_goal = PoseStamped()
    left_gripper_goal.header.frame_id = handle_name
    left_gripper_goal.pose.position.x = -0.01
    left_gripper_goal.pose.orientation.x = 1  # 180-degree rotation about x-axis of gripper
    giskard.set_cart_goal(goal_pose=left_gripper_goal, tip_link=left_gripper_name, root_link="base_footprint")
    giskard.plan_and_execute()
    control_gripper(open=False, left=True, right=False)

    # open drawer
    giskard.set_open_container_goal(tip_link=left_gripper_name, environment_link=handle_name, max_velocity=0.1)
    giskard.plan_and_execute()

    # close drawer
    giskard.set_close_container_goal(tip_link=left_gripper_name, environment_link=handle_name)
    giskard.plan_and_execute()
    control_gripper(open=True)

    # post grasp
    left_gripper_goal = PoseStamped()
    left_gripper_goal.header.frame_id = handle_name
    left_gripper_goal.pose.position.x = -0.1
    left_gripper_goal.pose.orientation.x = 1  # 180-degree rotation about x-axis of gripper
    giskard.set_cart_goal(goal_pose=left_gripper_goal, tip_link=left_gripper_name, root_link="base_footprint")
    giskard.plan_and_execute()
