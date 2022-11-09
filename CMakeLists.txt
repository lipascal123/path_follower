cmake_minimum_required(VERSION 3.0.2)
project(lonomy_path_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PACKAGE_DEPENDENCIES
  roscpp
  nav_msgs
  roslib # for ros::package::getPath()
  rowesys_navigation_msgs
  tf
  tf_conversions
  geometry_msgs
  visualization_msgs
  elevation_map_msgs
#  elevation_mapping_cupy
  eigen_conversions
  grid_map_core
  grid_map_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
message("Eigen Version:" ${EIGEN3_VERSION_STRING})
message("Eigen Path:" ${Eigen3_DIR})

#find_package(catkin COMPONENTS kindr)
#message("Eigen Version:" ${EIGEN3_VERSION_STRING})
#message("Eigen Path:" ${Eigen3_DIR})

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3
  INCLUDE_DIRS include
  LIBRARIES lonomy_path_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(
  ${PROJECT_NAME}
  src/lonomy_path_controller.cpp
  src/waypoint.cpp
        include/lonomy_path_controller/swath.h src/swath.cpp src/pp_controller.cpp)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/lonomy_path_controller_node.cpp
  src/waypoint.cpp
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)




