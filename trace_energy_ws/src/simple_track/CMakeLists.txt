cmake_minimum_required(VERSION 3.8)
project(stm32_simple_track)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(servo_message REQUIRED)
add_executable(simple_blob_vision src/simple_blob_vision.cpp)
ament_target_dependencies(simple_blob_vision
  rclcpp sensor_msgs std_msgs cv_bridge image_transport OpenCV
)

add_executable(simple_pd_servo src/simple_pd_servo.cpp)
ament_target_dependencies(simple_pd_servo
  rclcpp std_msgs builtin_interfaces servo_message
)

install(TARGETS
  simple_blob_vision
  simple_pd_servo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

