cmake_minimum_required(VERSION 2.8)

project(drawing_functions)

find_package( OpenCV REQUIRED )
add_executable( drawing_functions drawing_functions.cpp )
target_link_libraries( drawing_functions ${OpenCV_LIBS} )
