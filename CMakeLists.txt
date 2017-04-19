cmake_minimum_required(VERSION 2.8.3)
project(sac_translators)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  sac_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED system filesystem date_time thread)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    sac_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${catkin_LIBRARY_DIRS}
)
#TODO :: clean this up later

#############
## Testing ##
#############

#if (CATKIN_ENABLE_TESTING)
#	find_package(rostest REQUIRED)
#	add_rostest_gtest(scorbot_ik_test test/scorbot_ik.test src/test/scorbot_ik_test.cpp)
#endif()

###########
## Nodes ##
###########

add_executable(scorbot_ik src/scorbot_ik.cpp)
target_link_libraries(scorbot_ik ${catkin_LIBRARIES})
