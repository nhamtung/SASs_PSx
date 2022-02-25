cmake_minimum_required(VERSION 3.0.2)
project(sass_psx)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  geometry_msgs
  agv_define
)

add_message_files(
  FILES
  psx_manual.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs geometry_msgs agv_define 
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

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
## Mark executables for installation
install(TARGETS psx_manual_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/ps4.launch
  launch/psx_control_manual.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES
  config/ps4.config.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)