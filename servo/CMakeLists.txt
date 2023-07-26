cmake_minimum_required(VERSION 0.0.1)
project(servo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
)

add_message_files(FILES
  Joints_sub.msg
  Joints_publish.msg
  trajctory.msg
  Trajectory.msg
  Status.msg
  Main.msg
  Virtual.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
  scripts/servo_demo.py
  scripts/servo_publisher.py
  scripts/servo_arduino.py
  scripts/main.py
  scripts/Processer.py
  scripts/save.py
  scripts/test.py
  scripts/controller.py
  scripts/Center.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(CATKIN_DEPENDS
    ros_tcp_endpoint
    message_runtime)


