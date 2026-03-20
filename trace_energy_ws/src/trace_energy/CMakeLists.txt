cmake_minimum_required(VERSION 3.8)
project(trace_energy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(servo_message REQUIRED)

add_executable(read_image src/read_image.cpp)
add_executable(image_subscriber src/image_subscriber.cpp)
add_executable(trace_calculator src/trace_calculator.cpp)

target_include_directories(read_image PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(read_image PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  read_image
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
  "std_msgs"
  "image_transport"
  "OpenCV"
)

ament_target_dependencies(
  image_subscriber
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
  "std_msgs"
  "image_transport"
  "OpenCV"
)

ament_target_dependencies(
  trace_calculator
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
  "std_msgs"
  "image_transport"
  "OpenCV"
  "servo_message"
)

install(TARGETS read_image image_subscriber trace_calculator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
