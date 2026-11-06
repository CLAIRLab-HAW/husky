cmake_minimum_required(VERSION 3.5)
project(qb2_description)

find_package(ament_cmake REQUIRED)

# Data only: the generator expands urdf/ through $(find qb2_description).  No meshes yet -- the macro draws a box.
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
