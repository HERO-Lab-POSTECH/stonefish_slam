cmake_minimum_required(VERSION 3.8)
project(stonefish_slam)

# Default to C++17 (required for structured bindings in DDA batch processing)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

# Find OctoMap
find_package(octomap REQUIRED)

# Find OpenMP for parallel processing
find_package(OpenMP)

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
  ${OCTOMAP_INCLUDE_DIRS}
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
  stonefish_slam/nodes/dead_reckoning_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dead_reckoning_node
)

install(PROGRAMS
  stonefish_slam/nodes/feature_extraction_node_sim.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME feature_extraction_node_sim
)

install(PROGRAMS
  stonefish_slam/nodes/slam_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slam_node
)

install(PROGRAMS
  stonefish_slam/nodes/kalman_node.py
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

# DDA Traversal module (only needs Eigen)
if(pybind11_FOUND AND EIGEN3_FOUND)
  # Get Python include directories
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

  # Manual library creation for ROS2
  add_library(dda_traversal MODULE stonefish_slam/cpp/dda_traversal.cpp)

  target_link_libraries(dda_traversal PRIVATE
    pybind11::module
    pybind11::lto
    Eigen3::Eigen
    Python3::Module
  )

  target_include_directories(dda_traversal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stonefish_slam/cpp
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
  )

  set_target_properties(dda_traversal PROPERTIES
    PREFIX "${PYTHON_MODULE_PREFIX}"
    SUFFIX "${PYTHON_MODULE_EXTENSION}"
    CXX_VISIBILITY_PRESET "hidden"
  )

  # Install to Python package directory
  install(TARGETS dda_traversal
    LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  )

  message(STATUS "DDA Traversal module will be built with Python ${Python3_VERSION}")
else()
  message(WARNING "pybind11 or Eigen3 not found, DDA Traversal C++ module will not be built")
endif()

# Octree Mapping module (OctoMap-based)
if(pybind11_FOUND AND EIGEN3_FOUND AND octomap_FOUND)
  # Get Python include directories
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

  # Manual library creation for ROS2
  add_library(octree_mapping MODULE stonefish_slam/cpp/octree_mapping.cpp)

  target_link_libraries(octree_mapping PRIVATE
    pybind11::module
    pybind11::lto
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    Python3::Module
  )

  # Add OpenMP if available
  if(OpenMP_CXX_FOUND)
    target_link_libraries(octree_mapping PRIVATE OpenMP::OpenMP_CXX)
  endif()

  target_include_directories(octree_mapping PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stonefish_slam/cpp
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
  )

  set_target_properties(octree_mapping PROPERTIES
    PREFIX "${PYTHON_MODULE_PREFIX}"
    SUFFIX "${PYTHON_MODULE_EXTENSION}"
    CXX_VISIBILITY_PRESET "hidden"
  )

  # Install to Python package directory
  install(TARGETS octree_mapping
    LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  )

  message(STATUS "Octree Mapping module will be built with OctoMap and Python ${Python3_VERSION}")
else()
  message(WARNING "pybind11, Eigen3, or OctoMap not found, Octree Mapping C++ module will not be built")
endif()

# PCL module for fast ICP
if(libpointmatcher_FOUND AND PCL_FOUND AND pybind11_FOUND)
  message(STATUS "Building PCL module with libpointmatcher and PCL")

  # Get Python include directories
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

  add_library(pcl_module MODULE stonefish_slam/cpp/pcl.cpp)

  target_link_libraries(pcl_module PRIVATE
    pybind11::module
    pybind11::lto
    ${libpointmatcher_LIBRARIES}
    ${PCL_LIBRARIES}
    Python3::Module
  )

  target_include_directories(pcl_module PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stonefish_slam/cpp
    ${libpointmatcher_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
  )

  set_target_properties(pcl_module PROPERTIES
    PREFIX ""
    OUTPUT_NAME "pcl"
    SUFFIX ".so"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CXX_VISIBILITY_PRESET "hidden"
  )

  install(TARGETS pcl_module
    LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}/cpp
  )

  message(STATUS "PCL module will be built with libpointmatcher and PCL ${PCL_VERSION}")
else()
  message(WARNING "libpointmatcher, PCL or pybind11 not found, PCL C++ module will not be built")
endif()

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
