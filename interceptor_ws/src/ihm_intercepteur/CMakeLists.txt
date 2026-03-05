cmake_minimum_required(VERSION 3.8)
project(ihm_intercepteur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# =====================
# Python nodes
# =====================
install(PROGRAMS ihm_intercepteur/ros_node_ihm.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/qt_ihm.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/qt_item.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/simulator_gps.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/mavros_simulator.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/zoomable_graphic_view.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/scale_bar_widget.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/leaflet_map_manager.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/process_reader.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ihm_intercepteur/launch_mavros.sh DESTINATION lib/${PROJECT_NAME})


# =====================================
# Installation des fichier de lancement
# =====================================

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# =========================================
# Importation des messages du GPS Ellipes D
# =========================================


#set (msg_files
#  "msg/SbgEkfEuler.msg"
#  "msg/SbgEkfNav.msg"	
#  "msg/SbgEkfStatus.msg"
#)

#========================
# Génération des messages
#========================

#Les messages listés après DEPENDENCIES sont les messages utilisés dans les messages du groupe de messages "msg_files"
#rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
#  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
#)


# =====================
# Tests
# ====================
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
