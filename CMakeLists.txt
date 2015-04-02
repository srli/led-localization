cmake_minimum_required(VERSION 2.8.3)
project(led_localization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY
   msg
   FILES
   beacons.msg
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

include_directories(
  #${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDEDIR}

)

## Declare a cpp library
# add_library(led_localization
#   src/${PROJECT_NAME}/led_localization.cpp
# )

## Declare a cpp executable
# add_executable(led_localization_node src/led_localization_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(led_localization_node led_localization_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(led_localization_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/led_finder.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES package.xml 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
