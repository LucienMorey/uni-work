Week 4 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. 
If you have any questions about the material please raise them in the next tutorial session.

Polymorphism - Ex01
------------------

Building upon solution in week 03 (ex03) 

* If not already done in quiz2, add a pure virtual function. 
* In the main create a Square and Circle and access them via a pointer to Shape to get their description

Polymorphism - Ex02
------------------

Building upon your previous solution.

* Create a vector of Shapes and use it to store 2 Triangles, 2 Rectangles and 3 Circles
* Write a program that computes the total area of shapes in the vector

Library Creation - Ex03
-----------------

Building upon your previous solution we will compile a static library.


* Modify your CMakeLists.txt be including the sections for generating static libraries using the sample [CMakeLists.txt](./starter/shape_lib/CMakeLists.txt) to create a shapes library

If you have sudo access on your computer the install process will install your library to default install locations which on a Linux system is `/usr/local`

Alternative, you can specify to be any directory of choise and we have two examples below that can be uncommented
* 1. Installs to your build directory `set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})`
* 2. This would install to your home folder and within subfolder of project_name (ie /home/user/shape_library) `set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/${PROJECT_NAME}")`


To compile the executable
```bash
mkdir build
cd build
cmake ..
make
make install
```
Note where the install directory is.

Library Linking - Ex04
-----------------

Create a project using the `library_test` folder and CMakeLists.txt [project]{./starter/library_test/CMakeLists.txt) that links to your library static.

Modify the `CMakeLists.txt` and add the `include_directories` and `link_directories` if needed (if your library is not installed to be part of system` 

Modify the main (which uses the library) and

* Createa an vector of 4 Shapes and use it to store 2 Triangles and 2 Rectangles
* Accepets user input of an x,y location and computes an area of all shapes that intersect that point




