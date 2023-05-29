cmake_minimum_required(VERSION 3.1.3)
project(es5_with_hand_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  control_msgs
)

add_service_files(
  FILES
  moveitTrajectory.srv
)

generate_messages(
DEPENDENCIES 
control_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


catkin_install_python(PROGRAMS
  src/external_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/trajectory_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/moveit_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


