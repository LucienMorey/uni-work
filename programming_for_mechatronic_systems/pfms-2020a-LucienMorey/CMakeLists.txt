# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (PFMS-2020A-LUCIENMOREY)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
#add_subdirectory(tutorials/week01/starter week1)
#add_subdirectory(tutorials/week02/starter week2)
#add_subdirectory(tutorials/week03/starter week3)
#add_subdirectory(quizzes/quiz0/a quiz0/a)
#add_subdirectory(quizzes/quiz0/b quiz0/b)
#add_subdirectory(quizzes/quiz1/a quiz1/a)
#add_subdirectory(quizzes/quiz1/b quiz1/b)
#add_subdirectory(quizzes/quiz2/a quiz2/a)
#add_subdirectory(quizzes/quiz2/b quiz2/b)
#add_subdirectory(tutorials/week04/starter/shapes week4)
#add_subdirectory(tutorials/week04/starter/library_test week4libtest)
#add_subdirectory(quizzes/quiz3/a quiz3/a)
#add_subdirectory(quizzes/quiz3/b quiz3/b)
#add_subdirectory(quizzes/quiz4/a/b quiz4/a)
add_subdirectory(quizzes/quiz4/b quiz4/b)
add_subdirectory(quizzes/quiz5/a quiz5/a)
add_subdirectory(quizzes/quiz5/b quiz5/b)
add_subdirectory(tutorials/week06/starter week6)
add_subdirectory(tutorials/week07/starter/ex01 week7/e1)
#add_subdirectory(quizzes/quiz6/a quiz6/a)
#add_subdirectory(quizzes/quiz6/b quiz6/b)
#add_subdirectory(tutorials/week08/starter/unit_testing/ex01 week8/ut/1)
#add_subdirectory(tutorials/week08/starter/unit_testing/ex02 week8/ut/2)



