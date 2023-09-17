#!/usr/bin/env python3
from typing import List

import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PointStamped, Vector3Stamped
from tf.transformations import quaternion_from_matrix, quaternion_about_axis, rotation_matrix

from giskardpy.python_interface import GiskardWrapper
import giskardpy.utils.tfwrapper as tf

import os
import re
import rospy, rospkg

from multiverse_msgs.msg import ObjectAttribute
from multiverse_msgs.srv import Socket, SocketRequest, SocketResponse

from std_msgs.msg import Float64

from spawn_box import spawn_box
from spawn_milk_box import spawn_milk_box
from spawn_spoon import spawn_spoon

import multiverse_knowledge

import argparse

gripper_left_publishers = [
    rospy.Publisher("/gripper_left_left_finger_effort_controller/command", Float64, queue_size=10),
    rospy.Publisher("/gripper_left_right_finger_effort_controller/command", Float64, queue_size=10),
]

gripper_right_publishers = [
    rospy.Publisher("/gripper_right_left_finger_effort_controller/command", Float64, queue_size=10),
    rospy.Publisher("/gripper_right_right_finger_effort_controller/command", Float64, queue_size=10),
]

rospack = rospkg.RosPack()

def control_gripper(open: bool, left: bool = True, right: bool = True) -> None:
    if open:
        if left and right:
            rospy.loginfo("Open both grippers")
        elif left:
            rospy.loginfo("Open left gripper")
        else:
            rospy.loginfo("Open right gripper")
    else:
        if left and right:
            rospy.loginfo("Close both grippers")
        elif left:
            rospy.loginfo("Close left gripper")
        else:
            rospy.loginfo("Close right gripper")

    if left:
        for gripper_left_publisher in gripper_left_publishers:
            gripper_left_publisher.publish(Float64(100) if open else Float64(-60))

    if right:
        for gripper_right_publisher in gripper_right_publishers:
            gripper_right_publisher.publish(Float64(100) if open else Float64(-60))

class CRAM:
    map = "map"
    box_joints = ["box_flap_side_1_joint", "box_flap_side_2_joint"]
    box = "box"
    box_flap_side_1_link = "box_flap_side_1"
    box_flap_side_2_link = "box_flap_side_2"
    right_tip_link = "gripper_right_grasping_frame"
    left_tip_link = "gripper_left_grasping_frame"
    right_gripper_tool_frame = "right_gripper_tool_frame"
    left_gripper_tool_frame = "left_gripper_tool_frame"
    camera_link = "xtion_rgb_frame"
    base_footprint = "base_footprint"
    apartment = "apartment"
    torso_link = "torso_lift_link"
    fridge_handle = "fridge_door1_handle"
    drawer = "cabinet9_drawer2"
    drawer_handle = "cabinet9_drawer2_handle"
    fridge_door_tray = "fridge_door_tray1"
    park_pose = {
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
        "torso_lift_joint": 0.7,
    }
    left_park_pose = {
        "arm_left_1_joint": -1.0,
        "arm_left_2_joint": 0.0,
        "arm_left_3_joint": 1.5,
        "arm_left_4_joint": 2.2,
        "arm_left_5_joint": -1.5,
        "arm_left_6_joint": 0.5,
        "arm_left_7_joint": 0.0,
    }
    right_park_pose = {
        "arm_right_1_joint": -1.0,
        "arm_right_2_joint": 0.0,
        "arm_right_3_joint": 1.5,
        "arm_right_4_joint": 2.2,
        "arm_right_5_joint": -1.5,
        "arm_right_6_joint": 0.5,
        "arm_right_7_joint": 0.0,
    }
    left_pre_grasp_pose = {
        "arm_left_1_joint": 1.198,
        "arm_left_2_joint": -1.102,
        "arm_left_3_joint": 2.008,
        "arm_left_4_joint": 1.405,
        "arm_left_5_joint": -1.964,
        "arm_left_6_joint": 1.358,
        "arm_left_7_joint": -0.545,
    }
    left_post_grasp_pose = {
        "arm_left_1_joint": 0.953,
        "arm_left_2_joint": -1.108,
        "arm_left_3_joint": 1.662,
        "arm_left_4_joint": 1.660,
        "arm_left_5_joint": -2.074,
        "arm_left_6_joint": 1.398,
        "arm_left_7_joint": -0.456,
    }
    apartment_joint_values = {}

    def __init__(self, object_name):
        self.giskard = GiskardWrapper(node_name="giskard")
        rospy.wait_for_service("/multiverse/query_data")
        self.query_service = rospy.ServiceProxy("/multiverse/query_data", Socket)
        self._box_request = SocketRequest()
        self.object = object_name
        for joint_name in self.box_joints:
            object_attr = ObjectAttribute()
            object_attr.object_name = joint_name
            object_attr.attribute_names = ["joint_rvalue"]
            self._box_request.receive.append(object_attr)

    def open_grippers(self):
        control_gripper(True)

    def open_left_gripper(self):
        control_gripper(True, left=True, right=False)

    def close_left_gripper(self):
        control_gripper(False, left=True, right=False)

    def close_grippers(self):
        control_gripper(False)

    def open_right_gripper(self):
        control_gripper(True, left=False, right=True)

    def close_right_gripper(self):
        control_gripper(False, left=False, right=True)

    def load_box(self):
        if self.box not in self.giskard.get_group_names():
            self.giskard.clear_world()
            self.giskard.add_urdf(name=self.box, urdf=rospy.get_param("box_description"), pose=tf.lookup_pose(self.map, self.box), parent_link=self.map)
        self.update_box_state()

    def load_apartment(self):
        if self.apartment not in self.giskard.get_group_names():
            apartment_pose = PoseStamped()
            apartment_pose.header.frame_id = self.map
            apartment_pose.pose.orientation.w = 1
            self.giskard.add_urdf(name=self.apartment, urdf=rospy.get_param("apartment_description"), pose=apartment_pose, parent_link=self.map)
        result = self.giskard.get_group_info(self.apartment)
        self.apartment_joint_values = {x.split("/")[1] : 0.0 for x in result.joint_state.name}
        self.update_apartment_state()

    def load_milk_box(self):
        milk_box_pose = tf.lookup_pose(self.map, self.object)
        self.giskard.add_box(name=self.object, size=[0.06, 0.095, 0.2], pose=milk_box_pose, parent_link=self.map)

    def load_spoon(self):
        if self.object not in self.giskard.get_group_names():
            spoon_pose = tf.lookup_pose(self.map, self.object)
            self.giskard.add_mesh(name=self.object, mesh="package://static_objects/spoon/meshes/obj/SM_Spoon.obj", pose=spoon_pose)

    def load_object(self):
        if self.object == "milk_box":
            self.load_milk_box()
        elif self.object == "spoon":
            self.load_spoon()

    def look_into_box(self):
        self.initial_pose()
        inside_box = PointStamped()
        inside_box.header.frame_id = self.box
        pointing_axis = Vector3Stamped()
        pointing_axis.header.frame_id = self.camera_link
        pointing_axis.vector.x = 1
        self.giskard.set_pointing_goal(goal_point=inside_box, tip_link=self.camera_link, root_link=self.base_footprint, pointing_axis=pointing_axis)
        self.giskard.plan_and_execute()

    def initial_pose(self):
        init_cart_goal = PoseStamped()
        init_cart_goal.header.frame_id = self.map
        init_cart_goal.pose.position = Point(1.97, 2.5, 0)
        init_cart_goal.pose.orientation.w = 1
        self.giskard.set_joint_goal(goal_state=self.park_pose)
        self.drive_to(init_cart_goal)

    def drive_to(self, goal_pose: PoseStamped):
        self.giskard.set_json_goal(constraint_type="DiffDriveBaseGoal", goal_pose=goal_pose, tip_link=self.base_footprint, root_link=self.map)
        self.giskard.plan_and_execute()

    def get_box_joint_state(self) -> dict:
        box_joint_state = {}
        try:
            query_response: SocketResponse = self.query_service(self._box_request)

            if len(query_response.receive) != len(self.box_joints):
                print(query_response)
            else:
                for i, joint_name in enumerate(self.box_joints):
                    joint_value = query_response.receive[i].data[0]
                    box_joint_state[joint_name] = joint_value

        except rospy.ServiceException as error:
            print(f"Service call failed: {error}")

        return box_joint_state

    def update_box_state(self):
        box_joint_state = self.get_box_joint_state()
        self.giskard.set_json_goal("SetSeedConfiguration", seed_configuration=box_joint_state, group_name="box")
        self.giskard.plan()

    def update_apartment_state(self):
        request = SocketRequest()
        for joint_name in self.apartment_joint_values.keys():
            object_attr = ObjectAttribute()
            object_attr.object_name = joint_name
            object_attr.attribute_names = ["joint_rvalue", "joint_tvalue"]
            request.receive.append(object_attr)

        try:
            query_response: SocketResponse = self.query_service(request)

            if len(query_response.receive) != len(self.apartment_joint_values):
                print(query_response)
                raise ValueError
            else:
                for object_data in query_response.receive:
                    joint_value = object_data.data[0]
                    joint_name = object_data.object_name
                    self.apartment_joint_values[joint_name] = joint_value

        except rospy.ServiceException as error:
            print(f"Service call failed: {error}")

        self.giskard.set_json_goal("SetSeedConfiguration", seed_configuration=self.apartment_joint_values, group_name=self.apartment)
        self.giskard.plan()

    def open_left_flap(self):
        self.initial_pose()
        self.update_box_state()

        # pre grasp
        left_pre_grasp_pose = PoseStamped()
        left_pre_grasp_pose.header.frame_id = self.box_flap_side_2_link
        left_pre_grasp_pose.pose.position.x = 0.40
        left_pre_grasp_pose.pose.position.z = 0.05
        box_R_gripper = np.array([[-1, 0, 0, 0], [0, 0, -1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        gripper_R_goal = rotation_matrix(np.pi / 4, [0, 0, 1])
        box_R_goal = np.dot(box_R_gripper, gripper_R_goal)
        left_pre_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(box_R_goal))
        self.giskard.set_cart_goal(goal_pose=left_pre_grasp_pose, root_link=self.base_footprint, tip_link=self.left_tip_link)
        self.giskard.plan_and_execute()

        # prise
        left_prise_pose = PoseStamped()
        left_prise_pose.header.frame_id = self.left_tip_link
        left_prise_pose.pose.position.y = -0.1
        left_prise_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=left_prise_pose, root_link=self.base_footprint, tip_link=self.left_tip_link)
        self.giskard.plan_and_execute()
        rospy.sleep(2)
        self.update_box_state()

        # slip
        left_slip_pose = PoseStamped()
        left_slip_pose.header.frame_id = self.left_tip_link
        left_slip_pose.pose.position.x = 0.03
        left_slip_pose.pose.position.y = -0.02
        left_slip_pose.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 6, [0, 0, 1]))
        self.giskard.set_cart_goal(goal_pose=left_slip_pose, root_link=self.base_footprint, tip_link=self.left_tip_link)
        self.giskard.plan_and_execute()
        self.update_box_state()

        # open
        self.giskard.set_open_container_goal(
            tip_link=self.left_tip_link, environment_link=self.box_flap_side_2_link, environment_group="box", goal_joint_state=np.pi / 4
        )
        self.giskard.plan_and_execute()
        self.update_box_state()

        # turn
        left_turn_pose = PoseStamped()
        left_turn_pose.header.frame_id = self.left_tip_link
        left_turn_pose.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 3, [0, 0, 1]))
        self.giskard.set_cart_goal(goal_pose=left_turn_pose, root_link=self.base_footprint, tip_link=self.left_tip_link)
        self.giskard.plan_and_execute()
        self.update_box_state()

        # push
        self.giskard.set_open_container_goal(
            tip_link=self.left_tip_link, environment_link=self.box_flap_side_2_link, environment_group=self.box, goal_joint_state=np.pi / 1.6
        )
        self.giskard.plan_and_execute()
        self.update_box_state()

    def open_right_flap(self):
        self.initial_pose()
        self.update_box_state()

        # pre grasp
        right_pre_grasp_pose = PoseStamped()
        right_pre_grasp_pose.header.frame_id = self.box_flap_side_1_link
        right_pre_grasp_pose.pose.position.x = -0.40
        right_pre_grasp_pose.pose.position.z = 0.05
        box_R_gripper = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        gripper_R_goal = rotation_matrix(-np.pi / 4, [0, 0, 1])
        box_R_goal = np.dot(box_R_gripper, gripper_R_goal)
        right_pre_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(box_R_goal))
        self.giskard.set_cart_goal(goal_pose=right_pre_grasp_pose, root_link=self.base_footprint, tip_link=self.right_tip_link)
        self.giskard.plan_and_execute()

        # prise
        right_prise_pose = PoseStamped()
        right_prise_pose.header.frame_id = self.right_tip_link
        right_prise_pose.pose.position.y = 0.1
        right_prise_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=right_prise_pose, root_link=self.base_footprint, tip_link=self.right_tip_link)
        self.giskard.plan_and_execute()
        rospy.sleep(2)
        self.update_box_state()

        # slip
        right_slip_pose = PoseStamped()
        right_slip_pose.header.frame_id = self.right_tip_link
        right_slip_pose.pose.position.x = 0.03
        right_slip_pose.pose.position.y = 0.02
        right_slip_pose.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 6, [0, 0, 1]))
        self.giskard.set_cart_goal(goal_pose=right_slip_pose, root_link=self.base_footprint, tip_link=self.right_tip_link)
        self.giskard.plan_and_execute()

        # open
        self.giskard.set_open_container_goal(
            tip_link=self.right_tip_link, environment_link=self.box_flap_side_1_link, environment_group=self.box, goal_joint_state=np.pi / 4
        )
        self.giskard.plan_and_execute()
        self.update_box_state()

        # turn
        right_turn_pose = PoseStamped()
        right_turn_pose.header.frame_id = self.right_tip_link
        right_turn_pose.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 3, [0, 0, 1]))
        self.giskard.set_cart_goal(goal_pose=right_turn_pose, root_link=self.base_footprint, tip_link=self.right_tip_link)
        self.giskard.plan_and_execute()
        self.update_box_state()

        # push
        self.giskard.set_open_container_goal(
            tip_link=self.right_tip_link, environment_link=self.box_flap_side_1_link, environment_group="box", goal_joint_state=np.pi / 1.6
        )
        self.giskard.plan_and_execute()
        self.update_box_state()

    def pick_milk_box(self):
        pre_pick_pose = PoseStamped()
        pre_pick_pose.header.frame_id = self.object
        pre_pick_pose.pose.position.y = 0.45
        pre_pick_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=pre_pick_pose, root_link=self.base_footprint, tip_link=self.right_tip_link)
        self.giskard.plan_and_execute()

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.object
        grasp_pose.pose.position.y = 0.075
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=grasp_pose, root_link=self.torso_link, tip_link=self.right_tip_link)
        self.giskard.plan_and_execute()

        self.close_right_gripper()
        self.giskard.update_parent_link_of_group(name=self.object, parent_link=self.right_tip_link)

        post_pick_pose = PoseStamped()
        post_pick_pose.header.frame_id = self.object
        post_pick_pose.pose.position.y = 0.45
        post_pick_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=post_pick_pose, root_link=self.base_footprint, tip_link=self.object)
        self.giskard.plan_and_execute()

        post_pick_state = {}
        post_pick_state["arm_right_5_joint"] = -1.5
        post_pick_state["arm_right_6_joint"] = 0.5
        post_pick_state["arm_right_7_joint"] = 1.57
        self.giskard.set_joint_goal(goal_state=post_pick_state)
        self.giskard.plan_and_execute()

    def pick_spoon(self):
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = self.object
        pre_grasp_pose.pose.position.x = 0.075
        pre_grasp_pose.pose.position.z = 0.35
        pre_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))

        self.giskard.set_cart_goal(goal_pose=pre_grasp_pose, root_link=self.torso_link, tip_link=self.right_tip_link)
        self.giskard.set_joint_goal(goal_state=self.left_pre_grasp_pose)
        self.giskard.plan_and_execute()

        self.close_right_gripper()

        right_push_pose = PoseStamped()
        right_push_pose.header.frame_id = self.object
        right_push_pose.pose.position.x = 0.08
        right_push_pose.pose.position.z = 0.08
        right_push_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=right_push_pose, root_link=self.torso_link, tip_link=self.right_tip_link)

        left_grasp_pose = PoseStamped()
        left_grasp_pose.header.frame_id = self.object
        left_grasp_pose.pose.position.x = -0.03
        left_grasp_pose.pose.position.z = 0.1
        left_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=left_grasp_pose, root_link=self.torso_link, tip_link=self.left_tip_link)
        self.giskard.plan_and_execute()

        self.close_left_gripper()

        spoon_pose = tf.lookup_pose(self.map, self.object)
        self.giskard.update_group_pose(group_name=self.object, new_pose=spoon_pose)
        self.giskard.update_parent_link_of_group(name=self.object, parent_link=self.left_tip_link)

        right_post_push_pose = PoseStamped()
        right_post_push_pose.header.frame_id = self.right_tip_link
        right_post_push_pose.pose.position.x = -0.4
        right_post_push_pose.pose.position.y = 0.05
        right_post_push_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=right_post_push_pose, root_link=self.torso_link, tip_link=self.right_tip_link)
        self.giskard.set_joint_goal(self.left_post_grasp_pose)
        self.giskard.plan_and_execute()

        spoon_pose = tf.lookup_pose(self.map, self.object)
        self.giskard.update_group_pose(group_name=self.object, new_pose=spoon_pose)

        right_pre_grasp_pose = PoseStamped()
        right_pre_grasp_pose.header.frame_id = self.object
        right_pre_grasp_pose.pose.position.x = 0.23
        right_pre_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[-1, 0, 0, 0], [0, 0, -1, 0], [-0, -1, 0, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=right_pre_grasp_pose, root_link=self.torso_link, tip_link=self.right_tip_link)
        self.giskard.plan_and_execute()

        self.open_right_gripper()

        post_cart_goal = PoseStamped()
        post_cart_goal.header.frame_id = self.map
        post_cart_goal.pose.position.x = 1.7
        post_cart_goal.pose.position.y = 2.5
        post_cart_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=post_cart_goal, tip_link=self.base_footprint, root_link=self.map)
        self.giskard.plan_and_execute()

        spoon_pose = tf.lookup_pose(self.map, self.object)
        self.giskard.update_group_pose(group_name=self.object, new_pose=spoon_pose)

        right_grasp_pose = PoseStamped()
        right_grasp_pose.header.frame_id = self.object
        right_grasp_pose.pose.position.x = 0.1
        right_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[-1, 0, 0, 0], [0, 0, -1, 0], [-0, -1, 0, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=right_grasp_pose, root_link=self.torso_link, tip_link=self.right_tip_link, max_linear_velocity=0.1, max_angular_velocity=0.1)
        self.giskard.plan_and_execute()

        self.close_right_gripper()

        self.giskard.update_parent_link_of_group(name=self.object, parent_link=self.right_tip_link)

        rospy.sleep(1)

        self.open_left_gripper()

        left_pose_grasp_pose = PoseStamped()
        left_pose_grasp_pose.header.frame_id = self.left_tip_link
        left_pose_grasp_pose.pose.position.z = -0.2
        left_pose_grasp_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=left_pose_grasp_pose, root_link=self.torso_link, tip_link=self.left_tip_link)

        right_post_grasp_pose = PoseStamped()
        right_post_grasp_pose.header.frame_id = self.right_tip_link
        right_post_grasp_pose.pose.position.y = -0.15
        right_post_grasp_pose.pose.position.z = 0.15
        right_post_grasp_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=right_post_grasp_pose, root_link=self.torso_link, tip_link=self.right_tip_link)

    def pick_object(self):
        if object_name == "milk_box":
            self.pick_milk_box()
        elif object_name == "spoon":
            self.pick_spoon()

    def infer_container(self, containers: list):
        if "cabinet3" in containers:
            self.container = "cabinet3"
        elif "cabinet9_drawer2" in containers:
            self.container = "cabinet9_drawer2"

    def open_container(self):
        if self.container == "cabinet3":
            self.open_fridge()
        elif self.container == "cabinet9_drawer2":
            self.open_drawer()

    def open_fridge(self):
        self.open_left_gripper()
        self.giskard.set_joint_goal(goal_state=self.park_pose)
        pre_cart_goal = PoseStamped()
        pre_cart_goal.header.frame_id = self.map
        pre_cart_goal.pose.position = Point(1.25, 2.85, 0)
        pre_cart_goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))
        self.drive_to(pre_cart_goal)

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.fridge_handle
        grasp_pose.pose.position.x = -0.008
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=grasp_pose, tip_link=self.left_gripper_tool_frame, root_link=self.base_footprint)
        self.giskard.plan_and_execute()
        self.close_left_gripper()
        rospy.sleep(1)

        self.giskard.set_open_container_goal(
            tip_link=self.left_gripper_tool_frame, environment_link=self.fridge_handle, goal_joint_state=np.pi / 3
        )
        self.add_keep_base_position_goal()
        self.giskard.plan_and_execute()

    def open_drawer(self):
        pre_cart_goal = PoseStamped()
        pre_cart_goal.header.frame_id = self.map
        pre_cart_goal.pose.position.x = 1.7
        pre_cart_goal.pose.position.y = 2.5
        pre_cart_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=pre_cart_goal, tip_link=self.base_footprint, root_link=self.map)
        self.giskard.plan_and_execute()

        pre_pull_goal = PoseStamped()
        pre_pull_goal.header.frame_id = self.drawer_handle
        pre_pull_goal.pose.position.x = -0.1
        pre_pull_goal.pose.orientation.x = 1  # 180-degree rotation about x-axis of gripper
        self.giskard.set_cart_goal(goal_pose=pre_pull_goal, tip_link=self.left_gripper_tool_frame, root_link=self.base_footprint)
        self.giskard.plan_and_execute()

        pull_goal = PoseStamped()
        pull_goal.header.frame_id = self.drawer_handle
        pull_goal.pose.position.x = -0.01
        pull_goal.pose.orientation.x = 1  # 180-degree rotation about x-axis of gripper
        self.giskard.set_cart_goal(goal_pose=pull_goal, tip_link=self.left_gripper_tool_frame, root_link=self.base_footprint)
        self.giskard.plan_and_execute()
        self.close_left_gripper()

        self.giskard.set_open_container_goal(tip_link=self.left_gripper_tool_frame, environment_link=self.drawer_handle, max_velocity=0.1)
        self.giskard.plan_and_execute()

    def add_keep_base_position_goal(self):
        base_goal = PointStamped()
        base_goal.header.frame_id = self.base_footprint
        self.giskard.set_translation_goal(goal_point=base_goal, tip_link=self.base_footprint, root_link=self.map)

    def place_milk_box(self):
        milk_box_pose = tf.lookup_pose(self.right_tip_link, self.object)
        milk_box_pose.header.frame_id = f"tiago_dual/{milk_box_pose.header.frame_id}"
        self.giskard.update_group_pose(group_name=self.object, new_pose=milk_box_pose)
        
        place_pose = PoseStamped()
        place_pose.header.frame_id = self.fridge_door_tray
        place_pose.pose.position.y = -0.1
        place_pose.pose.position.z = 0.12
        place_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=place_pose, tip_link=self.object, root_link=self.map)
        self.add_keep_base_position_goal()

        left_place_goal = PoseStamped()
        left_place_goal.header.frame_id = self.left_gripper_tool_frame
        left_place_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=left_place_goal, tip_link=self.left_gripper_tool_frame, root_link=self.map)
        self.giskard.plan_and_execute()

        rotate_milk_goal = PoseStamped()
        rotate_milk_goal.header.frame_id = self.object
        rotate_milk_goal.pose.position.y = -0.02
        rotate_milk_goal.pose.position.z = -0.02
        rotate_milk_goal.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 2, [0, 1, 0]))
        self.giskard.set_cart_goal(goal_pose=rotate_milk_goal, tip_link=self.object, root_link=self.torso_link)
        self.giskard.plan_and_execute()
        self.open_right_gripper()
        self.giskard.update_parent_link_of_group(name=self.object, parent_link=self.fridge_handle)

        right_post_place_goal = PoseStamped()
        right_post_place_goal.header.frame_id = self.right_gripper_tool_frame
        right_post_place_goal.pose.position.x = -0.1
        right_post_place_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=right_post_place_goal, tip_link=self.right_gripper_tool_frame, root_link=self.base_footprint, max_angular_velocity=0.1)
        self.giskard.plan_and_execute()

    def place_spoon(self):
        place_goal = PoseStamped()
        place_goal.header.frame_id = self.drawer
        place_goal.pose.position.x = -0.1
        place_goal.pose.position.y = -0.1
        place_goal.pose.position.z = 0.3
        place_goal.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=place_goal, tip_link=self.right_gripper_tool_frame, root_link=self.torso_link)
        self.giskard.plan_and_execute()

        self.open_right_gripper()

        self.giskard.set_joint_goal(self.right_park_pose)
        self.giskard.plan_and_execute()

        spoon_pose = tf.lookup_pose(self.map, self.object)
        self.giskard.update_group_pose(group_name=self.object, new_pose=spoon_pose)
        self.giskard.update_parent_link_of_group(name=self.object, parent_link=self.drawer)

    def place_object(self):
        if object_name == "milk_box":
            self.place_milk_box()
        elif object_name == "spoon":
            self.place_spoon()

    def close_fridge(self):
        self.giskard.set_joint_goal(self.right_park_pose)
        self.giskard.plan_and_execute()

        self.update_apartment_state()
        self.giskard.set_close_container_goal(tip_link=self.left_gripper_tool_frame, environment_link=self.fridge_handle)
        self.add_keep_base_position_goal()
        self.giskard.plan_and_execute()
        self.open_left_gripper()

        push_goal = PoseStamped()
        push_goal.header.frame_id = self.left_gripper_tool_frame
        push_goal.pose.position.x = -0.05
        push_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=push_goal, tip_link=self.left_gripper_tool_frame, root_link=self.base_footprint)
        self.giskard.plan_and_execute()

        post_cart_goal = PoseStamped()
        post_cart_goal.header.frame_id = self.map
        post_cart_goal.pose.position = Point(1.5, 2.5, 0)
        post_cart_goal.pose.orientation.z = 1
        self.giskard.set_joint_goal(goal_state=self.park_pose)
        self.giskard.set_json_goal(constraint_type="DiffDriveBaseGoal", goal_pose=post_cart_goal, tip_link=self.base_footprint, root_link=self.map)
        self.giskard.plan_and_execute()

    def close_drawer(self):
        self.giskard.set_close_container_goal(tip_link=self.left_gripper_tool_frame, environment_link=self.drawer_handle)
        self.giskard.plan_and_execute()
        control_gripper(open=True)

        left_post_push_pose = PoseStamped()
        left_post_push_pose.header.frame_id = self.left_gripper_tool_frame
        left_post_push_pose.pose.position.x = -0.1
        left_post_push_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=left_post_push_pose, tip_link=self.left_gripper_tool_frame, root_link=self.torso_link)
        self.giskard.plan_and_execute()

    def close_container(self):
        if self.container == "cabinet3":
            self.close_fridge()
        elif self.container == "cabinet9_drawer2":
            self.close_drawer()

class Konclude:
    namespace = "https://ease-crc.org/ont/USD.owl"
    box_ABox_file_path = os.path.join(rospack.get_path("tiago_dual_in_apartment"), "ontology", "box", "box_ABox.owl")
    apartment_ABox_file_path = os.path.join(rospack.get_path("tiago_dual_in_apartment"), "ontology", "apartment", "apartment_ABox.owl")

    def __init__(self) -> None:
        pass

    def get_iri(self, object_name: str):
        return f"<{self.namespace}#{object_name}>"

    def assert_box_state(self, box_joint_state: dict) -> None:
        rospy.loginfo(f"Assert box state: {str(box_joint_state)}")
        print(f"Assert box state: {str(box_joint_state)}")
        with open(self.box_ABox_file_path, 'r') as file:
            content = file.read()
        
        for joint_name, joint_value in box_joint_state.items():
            pattern = r'(DataPropertyAssertion\(USD:hasJointValue USD:{}_jointValue ")([^"]+)(".*?\))'.format(joint_name)

            def replace_value(match):
                return match.group(1) + str(joint_value) + match.group(3)
            
            content = re.sub(pattern, replace_value, content)

        with open(self.box_ABox_file_path, 'w') as file:
            file.write(content)

    def is_box_flap_open(self, box_flap: str) -> bool:
        box_flap_iri = self.get_iri(box_flap)
        is_box_flap_open = multiverse_knowledge.isItAnOpenFlap(box_flap_iri, self.box_ABox_file_path)
        is_box_flap_closed = multiverse_knowledge.isItAClosedFlap(box_flap_iri, self.box_ABox_file_path)

        if is_box_flap_open == is_box_flap_closed:
            raise ValueError(f"Something is wrong with {box_flap_iri}, is that a box flap?")

        is_open = is_box_flap_open and not is_box_flap_closed
        if is_open:
            rospy.loginfo(f"{box_flap} is open")
        else:
            rospy.loginfo(f"{box_flap} is closed")

        return is_open
    
    def where_to_store_object(self, object_name: str) -> list:
        containers = []
        for container_iri in multiverse_knowledge.whereToStoreObject(self.get_iri(object_name), self.apartment_ABox_file_path):
            containers.append(container_iri.split('#')[-1].replace(">", ""))

        rospy.loginfo(f"{object_name} can be stored in {str(containers)}")
        return containers

def spawn_object(object_name: str) -> None:
    rospy.loginfo(f"Spawn {object_name}")
    if object_name == "milk_box":
        spawn_milk_box()
    elif object_name == "spoon":
        spawn_spoon()
    else:
        rospy.logwarn(f"object_name must be milk_box or spoon, not {object_name}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run box unpacking scenario")
    parser.add_argument("--spawn_object", type=str, default="milk_box", help="Object to spawn (milk_box or spoon)")
    args = parser.parse_args()
    object_name = args.spawn_object

    konclude = Konclude()

    rospy.init_node("demo")
    
    cram = CRAM(object_name)

    spawn_box()
    cram.load_box()

    cram.initial_pose()
    cram.open_grippers()

    spawn_object(object_name)

    konclude.assert_box_state(cram.get_box_joint_state())
        
    while not konclude.is_box_flap_open("box_flap_side_2") and not rospy.is_shutdown():
        cram.open_left_flap()
        konclude.assert_box_state(cram.get_box_joint_state())

    while not konclude.is_box_flap_open("box_flap_side_1") and not rospy.is_shutdown():
        cram.open_right_flap()
        konclude.assert_box_state(cram.get_box_joint_state())

    cram.look_into_box()
    cram.load_object() # Can be replaced by perception
    cram.initial_pose()

    cram.pick_object()

    cram.load_apartment()

    containers = konclude.where_to_store_object(object_name)
    cram.infer_container(containers)

    cram.open_container()
    cram.place_object()
    cram.close_container()

    cram.initial_pose()
