#!/usr/bin/env python3

import rospy
import actionlib
import sys
import moveit_commander
import geometry_msgs.msg
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

def action_server_callback(goal):
    # Perform actions based on the received goal
    moveit_commander.roscpp_initialize(sys.argv)  # Initialize MoveIt Commander

    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.1)

    # Move the robot to a specific pose
    pose_goal = geometry_msgs.msg.Pose()

    # Set the desired pose values in pose_goal
    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)

    # Provide feedback
    feedback = FollowJointTrajectoryFeedback()

    # Set the feedback fields in feedback message
    action_server.publish_feedback(feedback)

    # Provide the result
    result = FollowJointTrajectoryResult()

    # Set the result fields in result message
    action_server.set_succeeded(result)

    move_group.stop()  # Stop the robot's motion
    moveit_commander.roscpp_shutdown()  # Shutdown MoveIt Commander

def main():
    # Initialize the ROS node
    print("Initializing node... ")
    rospy.init_node('custom_moveit_action_server')

    # Create the action server
    print("Initializing joint trajectory action server...")
    action_server = actionlib.SimpleActionServer('action', FollowJointTrajectoryAction, action_server_callback, False)
    action_server.start()

    # Spin to keep the script running
    rospy.spin()

if __name__ == "__main__":
    main()
