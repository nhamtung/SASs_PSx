cmake_minimum_required(VERSION 3.0.2)
project(sass_psx)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  #agv_define
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs agv_define sensor_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/sass_psx.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GSTREAMER_LIBRARIES})

add_executable(psx_manual_node src/psx_manual_node.cpp)
target_link_libraries(psx_manual_node ${PROJECT_NAME})
add_dependencies(psx_manual_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})