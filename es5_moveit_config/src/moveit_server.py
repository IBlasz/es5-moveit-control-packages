#!/usr/bin/env python3

#from es5_moveit_config.srv import moveitTrajectory
#from es5_moveit_config.srv import moveJointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
import rospy

def handle_trajectory_goal(request):
    trajectory = request.trajectory

    action_goal = FollowJointTrajectoryActionGoal()
    action_goal.goal.trajectory = trajectory

    pub.publish(action_goal)

    return moveitTrajectory(success=True)


def moveit_server():
    rospy.init_node('moveit_server')
    pub = rospy.Publisher('/external_trajectory_service', FollowJointTrajectoryActionGoal, queue_size=10)
    s = rospy.Service('/manipulator_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, handle_trajectory_goal)

    rospy.spin()
    

if __name__ == "__main__":
    moveit_server()
