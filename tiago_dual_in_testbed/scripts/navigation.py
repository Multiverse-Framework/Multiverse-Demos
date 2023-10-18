#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

def move_base_client(goal_pose):
    # Creates the SimpleActionClient, passing the type of action to the constructor
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started listening for goals
    client.wait_for_server()

    # Create a goal to send to the action server
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map" # Assuming you're using a map frame. Change if needed.
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal pose. This is just an example. Change to your required position and orientation
    goal.target_pose.pose = goal_pose.pose

    # Sends the goal to the action server
    client.send_goal(goal)

    # Waits for the server to finish performing the action
    client.wait_for_result()

    # Returns the result
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('move_base_client_py')

        poses = []
        poses.append(Pose(Point(0.5, 4.0, 0.0), Quaternion(0, 0, 0.707, 0.707)))
        poses.append(Pose(Point(-1.5, -1.0, 0.0), Quaternion(0, 0, 1, 0)))
        poses.append(Pose(Point(-1, 2.5, 0.0), Quaternion(0, 0, 0.707, 0.707)))
        poses.append(Pose(Point(5.0, 3.0, 0.0), Quaternion(0, 0, 0, 1)))
        poses.append(Pose(Point(2.0, 6.0, 0.0), Quaternion(0, 0, -0.707, 0.707)))
        poses.append(Pose(Point(-1.0, -4.0, 0.0), Quaternion(0, 0, 0, 1)))

        while not rospy.is_shutdown():
            for pose in poses:
                # Define your goal pose here. This is just an example.
                goal_pose = PoseStamped()
                goal_pose.pose = pose

                result = move_base_client(goal_pose)
                if result:
                    rospy.loginfo("Goal execution done!")
                else:
                    rospy.loginfo("Failed to reach the goal.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Action client terminated.")
