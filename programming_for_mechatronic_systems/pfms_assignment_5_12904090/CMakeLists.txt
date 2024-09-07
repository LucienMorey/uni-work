cmake_minimum_required(VERSION 3.0.2)
project(pfms_assignment_5_12904090)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  a4_setup
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  rosbag
)

add_definitions( -DANALYSIS=ON )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system signals)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

find_package(OpenCV)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES drawing_tools
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
## Declare a C++ library
add_library(${PROJECT_NAME}-image_processing src/image_processing.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-image_processing ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}-geometry_utility src/geometry_utility.cpp)
target_link_libraries(${PROJECT_NAME}-geometry_utility ${catkin_LIBRARIES})
## Declare a C++ library
add_library(${PROJECT_NAME}-path_tracking src/path_tracker.cpp src/pure_pursuit.cpp src/point_shoot.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-path_tracking ${catkin_LIBRARIES} ${PROJECT_NAME}-geometry_utility)

## Add cmake target dependencies of the executable
## same as for the library above
## This enforces generating messages before building these executables
add_dependencies(${PROJECT_NAME}-path_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}-main src/main.cpp src/navigator.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
## This enforces generating messages before building these executables
add_dependencies(${PROJECT_NAME}-main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-main ${catkin_LIBRARIES} ${PROJECT_NAME}-path_tracking ${PROJECT_NAME}-image_processing ${OpenCV_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp src/navigator.cpp)
if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME}-path_tracking ${PROJECT_NAME}-image_processing ${PROJECT_NAME}-geometry_utility ${OpenCV_LIBRARIES})
message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
endif()

