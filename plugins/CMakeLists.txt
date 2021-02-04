cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


#find_package(catkin REQUIRED)

#catkin_package()


find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eurobench_bms_msgs_and_srvs REQUIRED)
find_package(beast_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${eurobench_bms_msgs_and_srvs_INCLUDE_DIRS})


find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(contact SHARED ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")



