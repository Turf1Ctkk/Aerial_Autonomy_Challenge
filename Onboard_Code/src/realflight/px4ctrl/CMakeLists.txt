cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  traj_utils
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 
find_library(OSQP_LIBRARY NAMES osqp PATHS /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu)

if(NOT OSQP_LIBRARY)
  message(FATAL_ERROR "OSQP library not found. Please install libosqp or set OSQP_LIBRARY path.")
endif()

get_filename_component(OSQP_LIBRARY_DIR ${OSQP_LIBRARY} DIRECTORY)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/mpc_controller.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node quadrotor_msgs)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
  ${OSQP_LIBRARY}
)

set_target_properties(px4ctrl_node PROPERTIES
  BUILD_RPATH "${OSQP_LIBRARY_DIR}"
  INSTALL_RPATH "${OSQP_LIBRARY_DIR}"
)

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
