cmake_minimum_required(VERSION 3.0.2)
project(lste_oc_srfc)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(oc_srfc_proj src/oc_srfc_proj.cpp)

target_link_libraries(oc_srfc_proj
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
