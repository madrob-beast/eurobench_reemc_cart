cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(eurobench_reemc_cart)

find_package(catkin REQUIRED)

catkin_package()

#find_package(gazebo REQUIRED)
#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(contact SHARED ContactPlugin.cc)
#target_link_libraries(contact ${GAZEBO_LIBRARIES})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_subdirectory(${PROJECT_SOURCE_DIR}/plugins)
foreach(dir launch models)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)



