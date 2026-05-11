# ==============================================================================
# File: CMakeLists.txt
# Purpose: Build configuration for the C++ kinematics solver. It compiles the 
#          ik_solver_node.cpp file and links it against KDL and ROS 2 libraries.
# ==============================================================================

cmake_minimum_required(VERSION 3.8)
project(collaborative_arm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 and kinematics libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

# Build the C++ executable from the source file
add_executable(ik_solver_node src/ik_solver_node.cpp)

# Link the required libraries to the executable
ament_target_dependencies(ik_solver_node 
  rclcpp 
  geometry_msgs 
  sensor_msgs
  trajectory_msgs 
  kdl_parser 
  orocos_kdl
)

# Install the executable in the package's lib folder so it can be run
install(TARGETS
  ik_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()