# ==============================================================================
# File: CMakeLists.txt
# Purpose: Build configuration for the collaborative_arm_description package.
#          It defines dependencies and installation rules for the URDF/Xacro 
#          files, ensuring they are copied to the correct installation space.
# ==============================================================================

cmake_minimum_required(VERSION 3.8)
project(collaborative_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all required packages for building the description
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install the specified directories so ROS 2 can locate the description files 
# during runtime.
install(DIRECTORY
  urdf
  meshes
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