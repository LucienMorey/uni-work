# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz7a)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )


###################################################################################################################
#Please leave this section as it will attepmt to determine your version of Ubuntu and link to correct library
execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE RELEASE_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message ( STATUS "System in use is ${RELEASE_CODENAME}" )

if("${RELEASE_CODENAME}" STREQUAL "xenial")
    message ( STATUS "Xenial 16.04 in use" )
elseif("${RELEASE_CODENAME}" STREQUAL "bionic")
    message ( STATUS "Bionic 18.04 in use " )
else("${RELEASE_CODENAME}" STREQUAL "xenial")
    message ( STATUS "Your vesrion on Ubuntu ${RELEASE_CODENAME} is not supported" )
endif("${RELEASE_CODENAME}" STREQUAL "xenial")

set(CAR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep/${RELEASE_CODENAME})


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CAR_LIB_HOME}/include
)

message ( STATUS "Looking for libray in ${CAR_LIB_HOME}" )
link_directories(${CAR_LIB_HOME}/lib)


###################################################################################################################

# To add executables provide the executable name followed by a list of source files
add_executable(racing main.cpp display_race.cpp)
#Linking against the library and OpenCV
target_link_libraries( racing ${OpenCV_LIBS} car)
