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

ament_package()
