cmake_minimum_required(VERSION 3.5)
project(husky_bringup)

find_package(ament_cmake REQUIRED)

# Launch files only.  robot.yaml's platform.extras.launch names a package and a path inside its share: Clearpath's
# generator (clearpath_generator_robot 2.9.3) builds Package(launch.package) unconditionally, so an entry with an
# absolute path and no package dies on 'pkg_' + None -- hence a package, and launch/ installed into it.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# The octomap feed's one source stays in setup/scripts/: the offboard images COPY it from the husky-setup build context
# (husky/setup), and a move into this package would take that context and its pins along.
install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../setup/scripts/octomap_feed.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME octomap_feed
)

ament_package()
