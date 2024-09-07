cmake_minimum_required(VERSION 2.8)

project(access_pixels)

find_package( OpenCV REQUIRED )
add_executable( access_pixels access_pixels.cpp )
target_link_libraries( access_pixels ${OpenCV_LIBS} )
