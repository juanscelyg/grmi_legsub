cmake_minimum_required(VERSION 2.8.3)
project(legsub_can)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  geometry_msgs
  message_generation
  tf
  std_msgs
  nav_msgs
  visualization_msgs
  dynamic_reconfigure)

catkin_python_setup()

#add_message_files(
#  FILES
#  FloatStamped.msg
#  BoolStamped.msg
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/MurControlMixer.cfg
#)

catkin_install_python(PROGRAMS  scripts/legsub_can_parse.py
                                scripts/legsub_gamepad_parse.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
