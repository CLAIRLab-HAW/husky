cmake_minimum_required(VERSION 3.5)
project(rs16_description)

find_package(ament_cmake REQUIRED)

# Data only: the generator expands urdf/ through $(find rs16_description), RViz and the foxglove_bridge resolve
# package://rs16_description/meshes/ through this install.
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
