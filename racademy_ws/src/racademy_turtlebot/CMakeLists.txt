cmake_minimum_required(VERSION 3.8)
project(racademy_turtlebot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_relay
)

ament_package()
