cmake_minimum_required(VERSION 3.5)
project(husky_moveit_config)

find_package(ament_cmake REQUIRED)

# Data only: view-moveit (deploy/offboard/scripts) reads config/ through this install, in lite and in the viewer.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
