# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (shapes)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will now create the Makefiles that are needed to make an executable "hello" that requires "hello.cpp" 
#add_executable(shapes main.cpp shape.cpp rectangle.cpp triangle.cpp circle.cpp)

# Create the library, replace <library_name> with the name of library 
# Replace <cpp_files> with the files that form part of library
add_library(shapes STATIC shape.cpp rectangle.cpp circle.cpp triangle.cpp)
# Replace <header_files> with the header files that form part of library and need to be distributed, seperated by ";" like "simulator.h;types.h"
set_target_properties(shapes  PROPERTIES PUBLIC_HEADER "shape.h;rectangle.h;circle.h;triangle.h")


# If you have sudo access on your computer the install process will install your library to default install locations
# Which on a Linux system is /usr/local

# Alternative, you can specify to be any directory of choise and we have two examples below that can be uncommented
# 1. Below installs to your build directory
#set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
# 2. This would install to your home folder and within subfolder of project_name (ie /home/user/shape_library)
#set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/${PROJECT_NAME}")

install(TARGETS shapes DESTINATION lib PUBLIC_HEADER DESTINATION include)
