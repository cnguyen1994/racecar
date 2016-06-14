cmake_minimum_required(VERSION 2.8.3)
project(racecar)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate added messages and services with any dependencies listed here
add_message_files(
	FILES
	CMD.msg
	LOC.msg
	POINT.msg
	TRA.msg
)

generate_messages(
 DEPENDENCIES
  std_msgs
 )
catkin_package( 
	CATKIN_DEPENDS 
	message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(RealTimePID src/RealTimePID.cpp)
target_link_libraries(RealTimePID ${catkin_LIBRARIES})
add_dependencies(RealTimePID racecar_generate_messages_cpp)

add_executable(Serial src/Serial.cpp)
target_link_libraries(Serial ${catkin_LIBRARIES})
add_dependencies(Serial racecar_generate_messages_cpp)

add_executable(rrtstar src/rrts_main.cpp src/system_single_integrator.cpp src/kdtree.c src/rrts.hpp)
target_link_libraries(rrtstar ${catkin_LIBRARIES})
add_dependencies(rrtstar racecar_generate_messages_cpp)

## Declare a C++ library
# add_library(racecar
#   src/${PROJECT_NAME}/racecar.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(racecar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(racecar_node src/racecar_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(racecar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(racecar_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS racecar racecar_node
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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_racecar.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
