cmake_minimum_required(VERSION 3.8)
project(pidtest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(servo_message REQUIRED)

add_executable(pidtest_node src/pidtest_node.cpp)
target_compile_features(pidtest_node PUBLIC cxx_std_17)
ament_target_dependencies(
  pidtest_node
  rclcpp
  cv_bridge
  sensor_msgs
  std_msgs
  image_transport
  OpenCV
  servo_message
)

install(TARGETS pidtest_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
