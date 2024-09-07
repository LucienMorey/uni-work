# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (week4_ex04)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# If your library is not in the system, state where the include directories are located
# 1. Below will look for them in source directory and lib subfloder
# include_directories( ${CMAKE_SOURCE_DIR}/lib )
# 2. This would look for them in /home/studet/my_library/include
include_directories( "$ENV{HOME}/my_library/include" )

# If your library is not in the system, state where the include directories are located
# 1. Below will look for them in source directory and lib subfloder
# link_directories( ${CMAKE_SOURCE_DIR}/lib )
# 2. This would look for them in /home/studet/my_library/lib
link_directories( "$ENV{HOME}/my_library/lib" )

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable

add_executable(shapes_ex main.cpp)

#link the library (here we called it shapes, you do not need to specify the lib (library would be called libshapes)
target_link_libraries(shapes_ex shapes)
