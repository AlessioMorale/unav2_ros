cmake_minimum_required(VERSION 2.8.3)
project(unav2_hardware)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  unav2_msgs
  message_generation
  dynamic_reconfigure
  controller_manager
  diagnostic_updater
  geometry_msgs
  hardware_interface
  realtime_tools
  roslaunch
  roslint
  rosserial_server
  sensor_msgs
  std_msgs
  topic_tools
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread chrono)

find_package(Protobuf REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/UnavBridge.cfg
   cfg/UnavEncoder.cfg
   cfg/UnavMechanical.cfg
   cfg/UnavOperation.cfg
   cfg/UnavPID.cfg
   cfg/UnavSafety.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES unav2_hardware
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy
  unav2_msgs
  sensor_msgs
  std_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${GTEST_INCLUDE_DIRS}
)

protobuf_generate_cpp(PROTO_SRC PROTO_HEADER
  proto/messages.proto
  proto/configuration.proto
)

set_source_files_properties(${PROTO_HEADER} ${PROTO_SRC}
    PROPERTIES GENERATED TRUE)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node 
src/unav2_hardware.cpp
src/unav2.cpp
${PROTO_SRC}
${PROTO_HEADER}
)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME hardware_node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

catkin_add_gtest(gtest_test
test/test_main.cpp
test/test_robot_io.cpp
${PROTO_SRC}
${PROTO_HEADER}
)

target_link_libraries(gtest_test
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
