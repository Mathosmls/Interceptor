cmake_minimum_required(VERSION 3.8)
project(gz_sim_one)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------------
# Installation URDF
# -----------------------------
# Copie tous les fichiers du dossier urdf dans share/${PROJECT_NAME}/urdf
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
