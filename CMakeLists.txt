cmake_minimum_required(VERSION 2.8.3)
project(camera_republisher)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  mrs_lib
  image_transport
  )

#Find Eigen libraries
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES CameraRepublisher
  CATKIN_DEPENDS roscpp mrs_lib std_msgs image_transport
  DEPENDS Eigen
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_library(CameraRepublisher src/camera_republisher.cpp)

add_dependencies(CameraRepublisher
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(CameraRepublisher
  ${catkin_LIBRARIES}
  )
