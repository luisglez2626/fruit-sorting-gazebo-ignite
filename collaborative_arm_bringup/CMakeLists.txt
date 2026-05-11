# ==============================================================================
# File: CMakeLists.txt
# Purpose: Build configuration for the bringup package. It simply installs the 
#          launch directory so the master launch script is available to ROS 2.
# ==============================================================================

cmake_minimum_required(VERSION 3.8)
project(collaborative_arm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Installs the launch directory to the package's share folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()