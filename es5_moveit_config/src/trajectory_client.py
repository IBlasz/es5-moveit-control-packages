#!/usr/bin/env python3

import sys
import rospy
#from es_master.srv import moveJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

def trajectory_client():
    #rospy.wait_for_service('/external_trajectory_service')
    #try:
    #move_joint_trajectory = rospy.ServiceProxy('/external_trajectory_service', moveJointTrajectory)
    move_joint_trajectory = rospy.Publisher('/external_trajectory_service', JointTrajectory, queue_size=1)

    rospy.init_node('trajectory_client') 
    
    traj = JointTrajectory()
    traj.header.frame_id = 'frame'
    traj.header.stamp = rospy.Time.now()
    traj.points = [JointTrajectoryPoint([255.0,77.0,28.0,106.0,110.0,0.0], [0.0,0.0,0.0,0.0,0.0,0.0], np.radians([]), np.radians([]), rospy.Duration(3.0)), 
        JointTrajectoryPoint([250.0,70.0,20.0,100.0,105.0,0.0], [0.0,0.0,0.0,0.0,0.0,0.0], np.radians([]), np.radians([]), rospy.Duration(3.0))]
    print("sending request...")
    #resp1 = move_joint_trajectory(traj)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move_joint_trajectory.publish(traj)
        # rospy.spinOnce()
        rate.sleep()
    #print(resp1.message)
    #return resp1.success
    #except rospy.ServiceException as e:
        #print("Service call failed: %s"%e)
    # rospy.spin()

if __name__ == "__main__":
    trajectory_client()
