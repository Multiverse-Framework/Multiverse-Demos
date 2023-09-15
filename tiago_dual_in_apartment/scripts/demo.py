#!/usr/bin/env python3
from typing import List, Dict

import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PointStamped, Vector3Stamped
from tf.transformations import quaternion_from_matrix, quaternion_about_axis, rotation_matrix

from giskardpy.python_interface import GiskardWrapper
from urdf_parser_py import urdf
import giskardpy.utils.tfwrapper as tf

import rospy
import rospkg

from multiverse_msgs.msg import ObjectAttribute
from multiverse_msgs.srv import Socket, SocketRequest, SocketResponse

from std_msgs.msg import Float64

from spawn_box import spawn_box
from spawn_milk_box import spawn_milk_box
from spawn_spoon import spawn_spoon

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
            gripper_left_publisher.publish(Float64(100) if open else Float64(-60))

    if right:
        for gripper_right_publisher in gripper_right_publishers:
            gripper_right_publisher.publish(Float64(100) if open else Float64(-60))


class CRAM:
    box_joints = ["box_flap_side_1_joint", "box_flap_side_2_joint"]
    box = "box"
    milk_box = "milk_box"
    spoon = "spoon"
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
    apartment_joints: List[str] = []
    fridge_handle_link = "fridge_door1_handle"
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
    right_park_pose = {
        "arm_right_1_joint": -1.0,
        "arm_right_2_joint": 0.0,
        "arm_right_3_joint": 1.5,
        "arm_right_4_joint": 2.2,
        "arm_right_5_joint": -1.5,
        "arm_right_6_joint": 0.5,
        "arm_right_7_joint": 0.0,
    }
    left_push_spoon_pose = {
        "torso_lift_joint": 0.78,
        "arm_right_1_joint": 1.498214,
        "arm_right_2_joint": -0.000614,
        "arm_right_3_joint": 2.855332,
        "arm_right_4_joint": 0.811665,
        "arm_right_5_joint": -1.850122,
        "arm_right_6_joint": 0.812098,
        "arm_right_7_joint": 1.324703,
    }

    def __init__(self):
        self.giskard = GiskardWrapper(node_name="giskard")
        rospy.wait_for_service("/multiverse/query_data")
        self.query_service = rospy.ServiceProxy("/multiverse/query_data", Socket)
        self._box_request = SocketRequest()
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
            box_pose = tf.lookup_pose("map", self.box)
            print(self.giskard.add_urdf(name=self.box, urdf=rospy.get_param("box_description"), pose=box_pose, parent_link="map"))
        self.update_box_state()

    def load_apartment(self):
        if self.apartment not in self.giskard.get_group_names():
            apartment_pose = PoseStamped()
            apartment_pose.header.frame_id = "map"
            apartment_pose.pose.orientation.w = 1
            self.giskard.add_urdf(name=self.apartment, urdf=rospy.get_param("apartment_description"), pose=apartment_pose, parent_link="map")
        result = self.giskard.get_group_info(self.apartment)
        self.apartment_joints = [x.split("/")[1] for x in result.joint_state.name]
        self.update_apartment_state()

    def load_milk_box(self):
        milk_pose = tf.lookup_pose("map", self.milk_box)
        self.giskard.add_box(name=self.milk_box, size=[0.06, 0.095, 0.2], pose=milk_pose, parent_link="map")

    def load_spoon(self):
        if self.spoon not in self.giskard.get_group_names():
            spoon_pose = tf.lookup_pose("map", self.spoon)
            self.giskard.add_urdf(name=self.spoon, urdf=rospy.get_param("spoon_description"), pose=spoon_pose, parent_link="map")

    def look_into_box(self):
        self.initial_pose()
        inside_box = PointStamped()
        inside_box.header.frame_id = self.box
        pointing_axis = Vector3Stamped()
        pointing_axis.header.frame_id = self.camera_link
        pointing_axis.vector.x = 1
        self.giskard.set_pointing_goal(goal_point=inside_box, tip_link=self.camera_link, root_link=self.base_footprint, pointing_axis=pointing_axis)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        

    def initial_pose(self):
        cart_goal = PoseStamped()
        cart_goal.header.frame_id = "map"
        cart_goal.pose.position = Point(1.97, 2.5, 0)
        cart_goal.pose.orientation.w = 1
        self.giskard.set_joint_goal(goal_state=self.park_pose)
        self.drive_to(cart_goal)

    def drive_to(self, goal_pose: PoseStamped):
        self.giskard.set_json_goal(constraint_type="DiffDriveBaseGoal", goal_pose=goal_pose, tip_link=self.base_footprint, root_link="map")
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

    def update_box_state(self):
        joint_values = {}
        try:
            query_response: SocketResponse = self.query_service(self._box_request)

            if len(query_response.receive) != len(self.box_joints):
                print(query_response)
                print("WTF")
            else:
                for i, joint_name in enumerate(self.box_joints):
                    joint_value = query_response.receive[i].data[0]
                    joint_values[joint_name] = joint_value

        except rospy.ServiceException as error:
            print(f"Service call failed: {error}")

        self.giskard.set_json_goal("SetSeedConfiguration", seed_configuration=joint_values, group_name="box")
        self.giskard.allow_all_collisions()
        self.giskard.plan()

    def update_apartment_state(self):
        request = SocketRequest()
        for joint_name in self.apartment_joints:
            object_attr = ObjectAttribute()
            object_attr.object_name = joint_name
            object_attr.attribute_names = ["joint_rvalue", "joint_tvalue"]
            request.receive.append(object_attr)
        joint_values = {}
        try:
            query_response: SocketResponse = self.query_service(request)

            if len(query_response.receive) != len(self.apartment_joints):
                print(query_response)
                print("WTF")
            else:
                for object_data in query_response.receive:
                    joint_value = object_data.data[0]
                    joint_name = object_data.object_name
                    joint_values[joint_name] = joint_value

        except rospy.ServiceException as error:
            print(f"Service call failed: {error}")

        self.giskard.set_json_goal("SetSeedConfiguration", seed_configuration=joint_values, group_name=self.apartment)
        self.giskard.allow_all_collisions()
        self.giskard.plan()

    def open_left_flap(self):
        tip_link = self.left_tip_link
        self.initial_pose()
        self.update_box_state()

        # pre grasp
        left_grasp_pose = PoseStamped()
        left_grasp_pose.header.frame_id = self.box_flap_side_2_link
        left_grasp_pose.pose.position.x = 0.40
        left_grasp_pose.pose.position.z = 0.05
        box_R_gripper = np.array([[-1, 0, 0, 0], [0, 0, -1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        gripper_R_goal = rotation_matrix(np.pi / 4, [0, 0, 1])
        box_R_goal = np.dot(box_R_gripper, gripper_R_goal)
        left_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(box_R_goal))
        self.giskard.set_cart_goal(goal_pose=left_grasp_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        # prise
        left_grasp_pose = PoseStamped()
        left_grasp_pose.header.frame_id = tip_link
        left_grasp_pose.pose.position.y = -0.1
        left_grasp_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=left_grasp_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        rospy.sleep(2)
        self.update_box_state()

        # slip
        left_grasp_pose = PoseStamped()
        left_grasp_pose.header.frame_id = tip_link
        left_grasp_pose.pose.position.x = 0.03
        left_grasp_pose.pose.position.y = -0.02
        left_grasp_pose.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 6, [0, 0, 1]))
        self.giskard.set_cart_goal(goal_pose=left_grasp_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.update_box_state()

        # open
        self.giskard.set_open_container_goal(
            tip_link=tip_link, environment_link=self.box_flap_side_2_link, environment_group="box", goal_joint_state=np.pi / 4
        )
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.update_box_state()

        # turn
        left_grasp_pose = PoseStamped()
        left_grasp_pose.header.frame_id = tip_link
        left_grasp_pose.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 3, [0, 0, 1]))
        self.giskard.set_cart_goal(goal_pose=left_grasp_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.update_box_state()

        # push
        self.giskard.set_open_container_goal(
            tip_link=tip_link, environment_link=self.box_flap_side_2_link, environment_group="box", goal_joint_state=np.pi / 1.6
        )
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.update_box_state()

    def open_right_flap(self):
        self.initial_pose()
        self.update_box_state()
        tip_link = self.right_tip_link

        # pre grasp
        right_grasp_pose = PoseStamped()
        right_grasp_pose.header.frame_id = self.box_flap_side_1_link
        right_grasp_pose.pose.position.x = -0.40
        right_grasp_pose.pose.position.z = 0.05
        box_R_gripper = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        gripper_R_goal = rotation_matrix(-np.pi / 4, [0, 0, 1])
        box_R_goal = np.dot(box_R_gripper, gripper_R_goal)
        right_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(box_R_goal))

        self.giskard.set_cart_goal(goal_pose=right_grasp_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        # prise
        right_grasp_pose = PoseStamped()
        right_grasp_pose.header.frame_id = tip_link
        right_grasp_pose.pose.position.y = 0.1
        right_grasp_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=right_grasp_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        rospy.sleep(2)
        self.update_box_state()

        # slip
        right_grasp_pose = PoseStamped()
        right_grasp_pose.header.frame_id = tip_link
        right_grasp_pose.pose.position.x = 0.03
        right_grasp_pose.pose.position.y = 0.02
        right_grasp_pose.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 6, [0, 0, 1]))
        self.giskard.set_cart_goal(goal_pose=right_grasp_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        # open
        self.giskard.set_open_container_goal(
            tip_link=tip_link, environment_link=self.box_flap_side_1_link, environment_group="box", goal_joint_state=np.pi / 4
        )
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.update_box_state()

        # turn
        right_grasp_pose = PoseStamped()
        right_grasp_pose.header.frame_id = tip_link
        right_grasp_pose.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 3, [0, 0, 1]))
        self.giskard.set_cart_goal(goal_pose=right_grasp_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.update_box_state()

        # push
        self.giskard.set_open_container_goal(
            tip_link=tip_link, environment_link=self.box_flap_side_1_link, environment_group="box", goal_joint_state=np.pi / 1.6
        )
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.update_box_state()

    def pick_milk_box(self):
        tip_link = self.right_tip_link
        pre_pose = PoseStamped()
        pre_pose.header.frame_id = self.milk_box
        pre_pose.pose.position.y = 0.45
        pre_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=pre_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.milk_box
        grasp_pose.pose.position.y = 0.075
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=grasp_pose, root_link=self.torso_link, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        self.close_right_gripper()
        self.giskard.update_parent_link_of_group(name=self.milk_box, parent_link=tip_link)

        pre_pose = PoseStamped()
        pre_pose.header.frame_id = self.milk_box
        pre_pose.pose.position.y = 0.45
        pre_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=pre_pose, root_link=self.base_footprint, tip_link=self.milk_box)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        goal_state = {}
        goal_state["arm_right_5_joint"] = -1.5
        goal_state["arm_right_6_joint"] = 0.5
        goal_state["arm_right_7_joint"] = 1.57
        self.giskard.set_joint_goal(goal_state=goal_state)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        self.initial_pose()

    def pick_spoon(self):
        tip_link = self.right_tip_link
        pre_pose = PoseStamped()
        pre_pose.header.frame_id = self.spoon
        pre_pose.pose.position.x = 0.055
        pre_pose.pose.position.z = 0.35
        pre_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        
        self.giskard.set_cart_goal(goal_pose=pre_pose, root_link=self.base_footprint, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.spoon
        grasp_pose.pose.position.x = 0.055
        grasp_pose.pose.position.z = 0.083
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=grasp_pose, root_link=self.torso_link, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        self.close_right_gripper()

        rospy.sleep(10)

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.spoon
        grasp_pose.pose.position.x = 0.055
        grasp_pose.pose.position.z = 0.1
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=grasp_pose, root_link=self.torso_link, tip_link=tip_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        # grasp_pose = PoseStamped()
        # grasp_pose.header.frame_id = self.spoon
        # grasp_pose.pose.position.x = 0.065
        # grasp_pose.pose.position.z = 0.08
        # grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        # self.giskard.set_cart_goal(goal_pose=grasp_pose, root_link=self.torso_link, tip_link=tip_link)
        # self.giskard.allow_all_collisions()
        # self.giskard.plan_and_execute()

        # self.close_right_gripper()

        # self.giskard.set_joint_goal(self.left_push_spoon_pose)
        # self.giskard.allow_all_collisions()
        # self.giskard.plan_and_execute()



        # grasp_pose = PoseStamped()
        # grasp_pose.header.frame_id = self.spoon
        # grasp_pose.pose.position.x = 0.1
        # grasp_pose.pose.position.z = 0.08
        # grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        # self.giskard.set_cart_goal(goal_pose=grasp_pose, root_link=self.torso_link, tip_link=tip_link)

        # self.close_right_gripper()
        # self.giskard.update_parent_link_of_group(name=self.spoon, parent_link=tip_link)

        # rospy.sleep(2)

        # pre_pose = PoseStamped()
        # pre_pose.header.frame_id = self.spoon
        # pre_pose.pose.position.z = 0.01
        # pre_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        # self.giskard.set_cart_goal(goal_pose=pre_pose, root_link=self.torso_link, tip_link=self.spoon)
        # self.giskard.allow_all_collisions()
        # self.giskard.plan_and_execute()

        # pre_pose = PoseStamped()
        # pre_pose.header.frame_id = self.spoon
        # pre_pose.pose.position.z = 0.045
        # pre_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        # self.giskard.set_cart_goal(goal_pose=pre_pose, root_link=self.torso_link, tip_link=self.spoon)
        # self.giskard.allow_all_collisions()
        # self.giskard.plan_and_execute()

        # goal_state = {}
        # goal_state["arm_right_5_joint"] = -1.5
        # goal_state["arm_right_6_joint"] = 0.5
        # goal_state["arm_right_7_joint"] = 1.57
        # self.giskard.set_joint_goal(goal_state=goal_state)
        # self.giskard.allow_all_collisions()
        # self.giskard.plan_and_execute()

        # self.initial_pose()

    def open_fridge(self):
        self.load_apartment()
        self.open_left_gripper()
        self.giskard.set_joint_goal(goal_state=self.park_pose)
        cart_goal = PoseStamped()
        cart_goal.header.frame_id = "map"
        cart_goal.pose.position = Point(1.25, 2.85, 0)
        cart_goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))
        self.drive_to(cart_goal)
        self.drive_to(cart_goal)

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.fridge_handle_link
        grasp_pose.pose.position.x = -0.008
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=grasp_pose, tip_link=self.left_gripper_tool_frame, root_link=self.base_footprint)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.close_left_gripper()
        rospy.sleep(1)

        self.giskard.set_open_container_goal(
            tip_link=self.left_gripper_tool_frame, environment_link=self.fridge_handle_link, goal_joint_state=np.pi / 3
        )
        self.add_keep_base_position_goal()
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

    def add_keep_base_position_goal(self):
        base_goal = PointStamped()
        base_goal.header.frame_id = self.base_footprint
        self.giskard.set_translation_goal(goal_point=base_goal, tip_link=self.base_footprint, root_link="map")

    def place_milk(self):
        self.update_milk_pose()
        fridge_door_tray1 = "fridge_door_tray1"
        place_pose = PoseStamped()
        place_pose.header.frame_id = fridge_door_tray1
        place_pose.pose.position.y = -0.1
        place_pose.pose.position.z = 0.12
        place_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])))
        self.giskard.set_cart_goal(goal_pose=place_pose, tip_link=self.milk_box, root_link="map")
        self.add_keep_base_position_goal()

        left_hand_goal = PoseStamped()
        left_hand_goal.header.frame_id = self.left_gripper_tool_frame
        left_hand_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=left_hand_goal, tip_link=self.left_gripper_tool_frame, root_link="map")
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        rotate_milk_goal = PoseStamped()
        rotate_milk_goal.header.frame_id = self.milk_box
        # TODO why y and not z?
        rotate_milk_goal.pose.position.y = -0.02
        rotate_milk_goal.pose.position.z = -0.02
        rotate_milk_goal.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 2, [0, 1, 0]))
        self.giskard.set_cart_goal(goal_pose=rotate_milk_goal, tip_link=self.milk_box, root_link=self.torso_link)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.open_right_gripper()
        self.giskard.update_parent_link_of_group(name=self.milk_box, parent_link=self.fridge_handle_link)

        right_goal = PoseStamped()
        right_goal.header.frame_id = self.right_gripper_tool_frame
        right_goal.pose.position.x = -0.07
        right_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=right_goal, tip_link=self.right_gripper_tool_frame, root_link=self.base_footprint)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

    def close_fridge(self):
        self.giskard.set_joint_goal(self.right_park_pose)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        self.update_apartment_state()
        self.giskard.set_close_container_goal(tip_link=self.left_gripper_tool_frame, environment_link=self.fridge_handle_link)
        self.add_keep_base_position_goal()
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()
        self.open_left_gripper()

        left_hand_goal = PoseStamped()
        left_hand_goal.header.frame_id = self.left_gripper_tool_frame
        left_hand_goal.pose.position.x = -0.05
        left_hand_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=left_hand_goal, tip_link=self.left_gripper_tool_frame, root_link=self.base_footprint)
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

        cart_goal = PoseStamped()
        cart_goal.header.frame_id = "map"
        cart_goal.pose.position = Point(1.5, 2.5, 0)
        cart_goal.pose.orientation.z = 1
        self.giskard.set_joint_goal(goal_state=self.park_pose)
        self.giskard.set_json_goal(constraint_type="DiffDriveBaseGoal", goal_pose=cart_goal, tip_link=self.base_footprint, root_link="map")
        self.giskard.allow_all_collisions()
        self.giskard.plan_and_execute()

    def update_milk_pose(self):
        milk_pose = tf.lookup_pose(self.right_tip_link, self.milk_box)
        milk_pose.header.frame_id = f"tiago_dual/{milk_pose.header.frame_id}"
        self.giskard.update_group_pose(group_name=self.milk_box, new_pose=milk_pose)


if __name__ == "__main__":
    rospy.init_node("demo")

    # spawn_box()
    # spawn_milk_box()
    spawn_spoon()

    cram = CRAM()
    # cram.load_box()
    cram.initial_pose()
    cram.open_grippers()
    # cram.open_left_flap()
    # cram.open_right_flap()
    # cram.look_into_box()
    # cram.load_milk_box()
    # cram.pick_milk_box()
    # cram.giskard.clear_world()
    cram.load_spoon()
    cram.pick_spoon()
    # cram.open_fridge()
    # cram.place_milk()
    # cram.close_fridge()
    # cram.initial_pose()
