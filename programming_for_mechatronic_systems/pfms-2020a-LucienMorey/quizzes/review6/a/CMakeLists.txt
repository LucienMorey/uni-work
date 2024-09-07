# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz_6a)

# Needed to use threads
find_package (Threads)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(RADAR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${RADAR_LIB_HOME})
message ( STATUS "Looking for libray in ${RADAR_LIB_HOME}" )
link_directories(${RADAR_LIB_HOME})

# Add executable
add_executable(radar_test main.cpp)

# Link the library
target_link_libraries(radar_test radar)

