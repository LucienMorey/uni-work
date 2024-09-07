# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz_4a)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will now create the Makefiles that are needed to make library "shape"
add_library(shapes circle.cpp shape.cpp)

# CMake will now create the Makefiles that are needed to make executable "shape_line_test"
add_executable(shape_line_test main.cpp line.cpp analysis.cpp)

# CMake will link the executable "shape_line" against library "shapes"
target_link_libraries(shape_line_test shapes)
