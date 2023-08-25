#!/usr/bin/env python3

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.enums import Arms
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot

world = BulletWorld()
robot = Object("tiago_dual", "tiago_dual", "tiago_dual.urdf", pose=Pose([1, 2, 0]))

spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([2.4, 2.2, 0.85]))
apartment = Object("apartment", "environment", "apartment.urdf")
apartment.attach(spoon, 'cabinet10_drawer1')

robot_desig = BelieveObject(names=["tiago_dual"])
apartment_desig = BelieveObject(names=["apartment"])
spoon_desig = BelieveObject(names=["spoon"])


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    handle_desig = ObjectPart(names=["cabinet10_drawer1_handle"], part_of=apartment_desig.resolve())
    torso_pose = 0.2
    torso_desig = MoveTorsoAction([torso_pose]).resolve().perform()
    drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(), robot_desig=robot_desig.resolve()).resolve()
    NavigateAction([drawer_open_loc.pose]).resolve().perform()
    OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    spoon.detach(apartment)
    PickUpAction(spoon_desig, ["right"], ["top"]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([0.15]).resolve().perform()
    spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
    placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()
    NavigateAction([placing_loc.pose]).resolve().perform()
    PlaceAction(spoon_desig, [spoon_target_pose], ["right"]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
