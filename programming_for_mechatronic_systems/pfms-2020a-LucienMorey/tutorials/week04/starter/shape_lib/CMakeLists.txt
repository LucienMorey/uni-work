cmake_minimum_required(VERSION 2.8.3)
project(shape_library)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Create the library, replace <library_name> with the name of library 
# Replace <cpp_files> with the files that form part of library
add_library(<library_name> STATIC <cpp_files>)
# Replace <header_files> with the header files that form part of library and need to be distributed, seperated by ";" like "simulator.h;types.h"
set_target_properties(<library_name>  PROPERTIES PUBLIC_HEADER "<header_files>")


# If you have sudo access on your computer the install process will install your library to default install locations
# Which on a Linux system is /usr/local

# Alternative, you can specify to be any directory of choise and we have two examples below that can be uncommented
# 1. Below installs to your build directory
#set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
# 2. This would install to your home folder and within subfolder of project_name (ie /home/user/shape_library)
#set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/${PROJECT_NAME}")

install(TARGETS <library_name> DESTINATION lib PUBLIC_HEADER DESTINATION include)
