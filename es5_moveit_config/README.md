# es5_moveit_config
This is basic MoveIt package that allows you to control ES5 robot.

To simulate a robot in Gazebo, use command
```bash
$ roslaunch es5_moveit_config gazebo.launch
```
then open another terminal and use command 
```bash
$ roslaunch es5_moveit_config es5_planning_execution.launch
```
This package contains a custom MoveIt action server to send informations about generated trajectory to robot
(still work in progress)

To run action server, use command
```bash
$ rosrun es5_moveit_config es5_joint_trajectory_action_server.py
```
then you can execute trajectory by using a es5_planning_execution.launch file.
