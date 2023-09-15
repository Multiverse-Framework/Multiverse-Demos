#!/usr/bin/env python3

import rospy

from mujoco_msgs.msg import ObjectStatus, ObjectInfo
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest
from std_srvs.srv import Trigger, TriggerRequest
from urdf_parser_py import urdf

import rospkg


def spawn_spoon() -> None:
    rospack = rospkg.RosPack()

    object_status_1 = ObjectStatus()
    object_status_1.info.name = "spoon_box"
    object_status_1.info.type = ObjectInfo.CUBE
    object_status_1.info.movable = False
    object_status_1.info.rgba.r = 0.9
    object_status_1.info.rgba.g = 0.9
    object_status_1.info.rgba.b = 0.9
    object_status_1.info.rgba.a = 1.0
    object_status_1.info.size.x = 0.1
    object_status_1.info.size.y = 0.15
    object_status_1.info.size.z = 0.1
    object_status_1.pose.position.x = 2.5
    object_status_1.pose.position.y = 2.5
    object_status_1.pose.position.z = 1.034
    object_status_1.pose.orientation.w = 1

    object_status_2 = ObjectStatus()
    object_status_2.info.name = "spoon"
    object_status_2.info.type = ObjectInfo.MESH
    object_status_2.info.mesh = rospack.get_path("static_objects") + "/spoon/mjcf/spoon.xml"
    object_status_2.info.movable = True
    object_status_2.pose.position.x = 2.45
    object_status_2.pose.position.y = 2.5
    object_status_2.pose.position.z = 1.144
    object_status_2.pose.orientation.x = 0.0
    object_status_2.pose.orientation.y = 0.0
    object_status_2.pose.orientation.z = -0.707
    object_status_2.pose.orientation.w = 0.707

    objects = SpawnObjectRequest()
    objects.objects = [object_status_1, object_status_2]
    rospy.wait_for_service("/mujoco/spawn_objects")
    try:
        spawn_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
        spawn_resp = spawn_objects(objects)
        rospy.loginfo("Spawn response: " + str(spawn_resp))

        spoon_urdf_path = rospack.get_path("static_objects") + "/spoon/urdf/spoon.urdf"
        spoon_urdf: urdf.Robot = urdf.Robot.from_xml_file(file_path=spoon_urdf_path)
        rospy.set_param("/spoon_description", spoon_urdf.to_xml_string())

    except rospy.ServiceException as error:
        print(f"Service call failed: {error}")


def reset_simulation():
    rospy.wait_for_service("/mujoco/reset")
    try:
        reset_service = rospy.ServiceProxy("/mujoco/reset", Trigger)
        reset_service(TriggerRequest())
    except rospy.ServiceException as error:
        print(f"Service call failed: {error}")


if __name__ == "__main__":
    rospy.init_node("spawn_spoon")
    spawn_spoon()
