cmake_minimum_required(VERSION 3.8)
project(stonefish_slam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find pybind11
find_package(pybind11 REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find libpointmatcher
find_package(libpointmatcher QUIET)
if(NOT libpointmatcher_FOUND)
  message(WARNING "libpointmatcher not found, ICP functionality may be limited")
endif()

# Find PCL
find_package(PCL QUIET COMPONENTS common filters)
if(NOT PCL_FOUND)
  message(WARNING "PCL not found, point cloud processing may be limited")
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

if(libpointmatcher_FOUND)
  include_directories(${libpointmatcher_INCLUDE_DIRS})
endif()

if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()

# Python module installation
ament_python_install_package(${PROJECT_NAME})

# Install Python executable scripts
install(PROGRAMS
  stonefish_slam/dead_reckoning_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dead_reckoning_node
)

install(PROGRAMS
  stonefish_slam/feature_extraction_node_sim.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME feature_extraction_node_sim
)

install(PROGRAMS
  stonefish_slam/slam_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slam_node
)

install(PROGRAMS
  stonefish_slam/kalman_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME kalman_node
)

# Build pybind11 modules
# CFAR module (only needs Eigen)
if(pybind11_FOUND AND EIGEN3_FOUND)
  # Get Python include directories
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

  # Manual library creation for ROS2
  add_library(cfar MODULE stonefish_slam/cpp/cfar.cpp)

  target_link_libraries(cfar PRIVATE
    pybind11::module
    pybind11::lto
    Eigen3::Eigen
    Python3::Module
  )

  target_include_directories(cfar PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stonefish_slam/cpp
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
  )

  set_target_properties(cfar PROPERTIES
    PREFIX "${PYTHON_MODULE_PREFIX}"
    SUFFIX "${PYTHON_MODULE_EXTENSION}"
    CXX_VISIBILITY_PRESET "hidden"
  )

  # Install to Python package directory
  install(TARGETS cfar
    LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  )

  message(STATUS "CFAR module will be built with Python ${Python3_VERSION}")
else()
  message(WARNING "pybind11 or Eigen3 not found, CFAR C++ module will not be built")
endif()

# PCL module (disabled for now - needs libpointmatcher)
# if(libpointmatcher_FOUND AND PCL_FOUND AND pybind11_FOUND)
#   pybind11_add_module(pcl_module stonefish_slam/cpp/pcl.cpp)
#   target_link_libraries(pcl_module PRIVATE
#     ${libpointmatcher_LIBRARIES}
#     ${PCL_LIBRARIES}
#   )
#   install(TARGETS pcl_module
#     LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
#     RENAME pcl.so
#   )
# endif()

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
