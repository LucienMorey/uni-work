# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (week4_ex01-02)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# In this example we will create a library that will ONLY be used within this project
# Previously all cpp files would be compiled into the executable. 
# for instance our first executable would be
# add_executable(shapes-ex01 main-ex01.cpp shape.cpp rectangle.cpp triangle.cpp circle.cpp)
# add_executable(shapes-ex02 main-ex02.cpp shape.cpp rectangle.cpp triangle.cpp circle.cpp)


# Instead the process now is to (1) add library and then (2) link library

# Firstly, we can craete a library from a set of files, which simplifies compilation
#
# To create the library the Syntax is
# add_library(<library_name> STATIC <cpp_files>)
#
# Replace <library_name> with the name of library
# Replace <cpp_files> with the files that form part of library
#
# our libbary will be called shapes and the files that form the library are shapes.cpp (parent class shape) 
# ,recatngle.cpp and triangle.cpp (derived classes). We have no other helper classes here (that interact with them and could be included)
#
add_library(shapes STATIC shape.cpp circle.cpp rectangle.cpp triangle.cpp)
# the library will actually be "libshapes.a" (the system appends "lib" to the front and being a STATIC library extensions is ".a")


# Now our executables only have relevant files
#
# Ex01
add_executable(shapes-ex01 main-ex01.cpp)
# Ex02
add_executable(shapes-ex02 main-ex02.cpp)

# Second step now is to LINK against the library
# 
# To link against the library the Syntax is
# target_link_libraries(<executabl_name> <library_name>)

# Our executables were shapes-ex01 and shapes-ex02 and the library is shapes 
target_link_libraries(shapes-ex01 shapes)
target_link_libraries(shapes-ex02 shapes)
