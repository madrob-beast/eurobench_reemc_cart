cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(door_wind SHARED door_wind.cc)
target_link_libraries(door_wind ${GAZEBO_LIBRARIES})

add_library(simple_door_config SHARED simple_door_config.cc)
target_link_libraries(simple_door_config ${GAZEBO_LIBRARIES})

add_library(simple_door_config SHARED ContactPlugin.cc)
target_link_libraries(simple_door_config ${GAZEBO_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
