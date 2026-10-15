cmake_minimum_required(VERSION 3.0.2)
project(gazebo_click_point)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp std_msgs std_srvs)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_click_point src/click_point_gui_plugin.cpp)
target_link_libraries(gazebo_click_point ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} Qt5::Widgets)
