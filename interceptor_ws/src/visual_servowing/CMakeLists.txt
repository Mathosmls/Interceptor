cmake_minimum_required(VERSION 3.8)
project(visual_servowing)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# 2. Include Headers
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
# 3. Create the Executable (The Node)
add_executable(control_node_exec
  src/visual_control.cpp
)
# Color detection node (OpenCV)
add_executable(color_detector_node
  src/color_detector.cpp
)
# 4. Link Dependencies
ament_target_dependencies(control_node_exec
  rclcpp
  geometry_msgs
  vision_msgs
)
ament_target_dependencies(color_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
)

target_link_libraries(color_detector_node ${OpenCV_LIBRARIES})

target_link_libraries(control_node_exec Eigen3::Eigen)
# 5. Install Rules (Crucial for "ros2 run" and "ros2 launch")
# Install the C++ executable
install(TARGETS
  control_node_exec
  color_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install the Launch files (if present)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()
# Install the Header files (Good practice)
install(DIRECTORY include/
  DESTINATION include/
)
# install the config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(PROGRAMS
  scripts/calibrate_camera.py
  scripts/rectified_image_publisher.py
  scripts/byte_tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
