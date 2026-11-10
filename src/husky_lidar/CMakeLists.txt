cmake_minimum_required(VERSION 3.8)
project(husky_lidar)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

add_executable(self_filter src/self_filter.cpp)
target_compile_features(self_filter PRIVATE cxx_std_17)
target_link_libraries(self_filter
  geometric_shapes::geometric_shapes rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  tf2_eigen::tf2_eigen tf2_ros::tf2_ros urdf::urdf)
install(TARGETS self_filter DESTINATION lib/${PROJECT_NAME})

# The ROS-free modules (wiring, models, the drivers' configuration) that the launch files, mock and the feeds import.
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_body test/test_robot_body.cpp)
  target_include_directories(test_robot_body PRIVATE src)
  target_link_libraries(test_robot_body geometric_shapes::geometric_shapes rclcpp::rclcpp urdf::urdf)
endif()

ament_package()
