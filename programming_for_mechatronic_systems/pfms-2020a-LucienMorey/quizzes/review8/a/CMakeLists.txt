cmake_minimum_required (VERSION 3.2)
project(quiz_8a)
set(CMAKE_CXX_STANDARD 11)

add_executable(vector_ops vector_ops.cpp)

add_executable(shapes_ex main.cpp shape.cpp rectangle.cpp rectanglehelper.cpp)
