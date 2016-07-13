cmake_minimum_required(VERSION 2.8.3)
project(racecar)

find_package(ZED 1.0 REQUIRED)

option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()


##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
    SET(OCV_VERSION "2.4")
    SET(CUDA_VERSION "")
else()
    SET(CUDA_VERSION "7.5")
    SET(OCV_VERSION "3.1")
endif()

find_package(OpenCV ${OCV_VERSION} COMPONENTS core highgui imgproc REQUIRED)
find_package(CUDA ${CUDA_VERSION} REQUIRED)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  rosconsole
  sensor_msgs
  cv_bridge
  dynamic_reconfigure
  tf2_ros
  image_transport
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

generate_dynamic_reconfigure_options(
  cfg/Zed.cfg
)
catkin_package( 
	CATKIN_DEPENDS 
	roscpp
    	rosconsole
    	sensor_msgs
    	opencv
    	cv_bridge
    	image_transport
    	dynamic_reconfigure
    	tf2_ros
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
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(RealTimePID src/RealTimePID.cpp)
target_link_libraries(RealTimePID ${catkin_LIBRARIES})
add_dependencies(RealTimePID racecar_generate_messages_cpp)

add_executable(Serial src/Serial.cpp)
target_link_libraries(Serial ${catkin_LIBRARIES})
add_dependencies(Serial racecar_generate_messages_cpp)

add_executable(rrtstar src/rrts_main.cpp src/system_single_integrator.cpp src/kdtree.c src/rrts.hpp)
target_link_libraries(rrtstar ${catkin_LIBRARIES})
add_dependencies(rrtstar racecar_generate_messages_cpp)

add_definitions(-std=c++11)# -m64) #-Wall)

add_executable(cam_sub src/cam_sub.cpp)
target_link_libraries(cam_sub 
			${catkin_LIBRARIES}
			${ZED_LIBRARIES}
			${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
        		${OpenCV_LIBS}
       			${PCL_LIBRARIES}
	
)
add_dependencies(cam_sub ${PROJECT_NAME}_gencfg)


FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

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
