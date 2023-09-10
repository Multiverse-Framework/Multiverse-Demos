#!/usr/bin/env python3

import rospy

from mujoco_msgs.msg import ObjectStatus, ObjectInfo
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest
from std_srvs.srv import Trigger, TriggerRequest
from urdf_parser_py import urdf

import rospkg


def spawn_box() -> None:
    rospack = rospkg.RosPack()

    object_status = ObjectStatus()
    object_status.info.name = "box"
    object_status.info.type = ObjectInfo.MESH
    object_status.info.movable = False
    object_status.info.mesh = rospack.get_path("articulated_objects") + "/box/mjcf/box.xml"
    object_status.info.size.x = 1
    object_status.info.size.y = 1
    object_status.info.size.z = 1
    object_status.pose.position.x = 2.5
    object_status.pose.position.y = 2.5
    object_status.pose.position.z = 1.104
    object_status.pose.orientation.x = 0.0
    object_status.pose.orientation.y = 0.0
    object_status.pose.orientation.z = 0.707
    object_status.pose.orientation.w = 0.707

    objects = SpawnObjectRequest()
    objects.objects = [object_status]
    rospy.wait_for_service("/mujoco/spawn_objects")
    try:
        spawn_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
        spawn_resp = spawn_objects(objects)
        rospy.loginfo("Spawn response: " + str(spawn_resp))

        box_urdf_path = rospack.get_path("articulated_objects") + "/box/urdf/box.urdf"
        box_urdf: urdf.Robot = urdf.Robot.from_xml_file(file_path=box_urdf_path)
        rospy.set_param("/box_description", box_urdf.to_xml_string())

    except rospy.ServiceException as error:
        print(f"Service call failed: {error}")


if __name__ == "__main__":
    rospy.init_node("spawn_box")
    spawn_box()
