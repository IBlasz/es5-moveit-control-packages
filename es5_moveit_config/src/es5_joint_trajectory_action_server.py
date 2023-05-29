# from __future__ import print_function
# import argparse

import rospy
import actionlib

from dynamic_reconfigure.server import Server

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

class JointTrajectoryActionServer(object):
    def __init__(self):
        rate=100.0
        
        self._server = actionlib.SimpleActionServer(
            "/manipulator_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)
        self._action_name = rospy.get_name()
      
        self._server.start()
        
        # Action Feedback/Result
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()
     
        # Set joint state publishing to specified control rate
        self._pub_trajectory = rospy.Publisher(
            '/external_trajectory_service',
             JointTrajectory,
             queue_size=10)
        self._pub_trajectory.publish()

    # def _update_feedback(self, cmd_point, jnt_names, cur_time):
    #     self._fdbk.header.stamp = rospy.get_rostime()
    #     self._fdbk.joint_names = jnt_names
    #     self._fdbk.desired = cmd_point
    #     self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
    #     self._fdbk.actual.positions = self._get_current_position(jnt_names)
    #     self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
    #     self._fdbk.error.positions = list(map(operator.sub,
    #                                      self._fdbk.desired.positions,
    #                                      self._fdbk.actual.positions
    #                                     ))
    #     self._fdbk.error.time_from_start = rospy.Duration.from_sec(cur_time)
    #     self._server.publish_feedback(self._fdbk)


    def _command_joints(self, joint_names, point, start_time, dimensions_dict):
        if self._server.is_preempt_requested() or not self.robot_is_enabled():
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            self._command_stop(joint_names, self._limb.joint_angles(), start_time, dimensions_dict)
            return False
        velocities = []
        deltas = self._get_current_error(joint_names, point.positions)
        for delta in deltas:
            if ((math.fabs(delta[1]) >= self._path_thresh[delta[0]]
                and self._path_thresh[delta[0]] >= 0.0)) or not self.robot_is_enabled():
                rospy.logerr("%s: Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(self._result)
                self._command_stop(joint_names, self._limb.joint_angles(), start_time, dimensions_dict)
                return False
            if self._mode == 'velocity':
                velocities.append(self._pid[delta[0]].compute_output(delta[1]))
        if ((self._mode == 'position' or self._mode == 'position_w_id')
              and self._alive):
            cmd = dict(zip(joint_names, point.positions))
            raw_pos_mode = (self._mode == 'position_w_id')
            self._limb.set_joint_positions(cmd, raw=raw_pos_mode)
            if raw_pos_mode:
                ff_pnt = self._reorder_joints_ff_cmd(joint_names, point)
                self._pub_ff_cmd.publish(ff_pnt)
        elif self._alive:
            cmd = dict(zip(joint_names, velocities))
            self._limb.set_joint_velocities(cmd)
        return True


    def _on_trajectory_action(self, goal: FollowJointTrajectoryActionGoal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        
        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return
        rospy.loginfo("%s: Executing requested joint trajectory" %
                      (self._action_name,))
        rospy.logdebug("Trajectory Points: {0}".format(trajectory_points))

        # traj = JointTrajectory()
        # traj.header.frame_id = ''
        # traj.header.stamp = rospy.Time.now()
        # traj.points = goal.goal.trajectory.points

        self._pub_trajectory.publish(goal.trajectory)

        t = rospy.Rate(0.2)
        t.sleep()
        
        # self._update_feedback(deepcopy(last), joint_names,
        #                         now_from_start)
      
        # Verify goal constraint
        result = True
        if result is True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded" %
                          (self._action_name))
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold" %
                         (self._action_name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s" %
                         (self._action_name, result))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
            

def main():

    print("Initializing node... ")
    rospy.init_node("joint_trajectory_action_server")
    print("Initializing joint trajectory action server...")
    
    jtas = JointTrajectoryActionServer()

    print("Running. Ctrl-c to quit")
    rospy.spin()


if __name__ == "__main__":
    main()

