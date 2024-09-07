# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (my_library)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Create the library, replace <library_name> with the name of library
# Replace <cpp_files> with the files that form part of library
add_library(shapes STATIC rectangle.cpp circle.cpp triangle.cpp shape.cpp line.cpp)

# Replace <header_files> with the header files that form part of library and need to be distributed, seperated by ";" like "simulator.h;types.h"
set_target_properties(shapes  PROPERTIES PUBLIC_HEADER "rectangle.h;circle.h;triangle.h;shape.h;line.h")

# If you have sudo access on your computer the install process will install your library to default install locations
# Which on a Linux system is /usr/local

# Alternative, you can specify to be any directory of choise and we have two examples below that can be uncommented
# 1. Below installs to your build directory
#set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
# 2. This would install to your home folder and within subfolder of project_name (ie /home/user/my_library)
set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/${PROJECT_NAME}")

install(TARGETS shapes DESTINATION lib PUBLIC_HEADER DESTINATION include)

# From within this folder
# mkdir build
# cmake ..
# make
# make install
#
# if your installing it to the system path /usr/local the last step is
# sudo make install
