cmake_minimum_required(VERSION 3.8)
project(trace_energy)

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

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
include_directories(${Qt5_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 主链路：read_image → trace_calculator → servo_uart_node（见 trace_start.launch.py）
add_executable(read_image src/read_image.cpp)
add_executable(processed_image_preview src/processed_image_preview.cpp)
add_executable(trace_calculator src/trace_calculator.cpp)
add_executable(aim_control_panel src/aim_control_panel.cpp)

target_include_directories(read_image PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(read_image PUBLIC c_std_99 cxx_std_17)

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
  processed_image_preview
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
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

ament_target_dependencies(
  aim_control_panel
  "rclcpp"
  "std_msgs"
)

target_link_libraries(aim_control_panel
  Qt5::Core
  Qt5::Widgets
)

target_link_libraries(trace_calculator
  ${rclcpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${image_transport_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

install(TARGETS read_image processed_image_preview trace_calculator aim_control_panel
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/tracking_debug_echo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tracking_debug_echo)

install(PROGRAMS scripts/hsv_range_tuner_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hsv_range_tuner_gui)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
