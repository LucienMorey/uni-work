Quiz 0
======

Part A
------
1) Attempt to compile the code in [folder a](./a). Why does the code fail to compile?
    * The code fails to compile because the printArray function was redefined in an attempt to overload the function. To do this successfully the arguements parsed need to be different in type or number. 

2) Fix the compilation error. Why does a segmentation fault (segfault) occur? 
    * The compilation error was fixed by renaming the second printArray function to printPointerArray. The segementation fault occured because values were being appended beyond the boundary of the array. 

3) Modify the code so it will run
    *I modified the local array decleration in the main have have ARRAY_MAX_SIZE in length. 

4) Create a function that prints only the elements of the array that are larger than : mean + one standard deviation
    *See function printOutlier in arrays.cpp

5) Create a function that assigns elements of array x to a vector named `vec` (HINT: decide on the correct type)
    *See function vectorAssigner

Part B
------
1) Look at the code in [folder b](./b). Implement the methods of the Sample class in the [sample.cpp](./b/sample.cpp) file based on the definition provided in [sample.h](./b/sample.h)
    *See sample.cpp

2) Make an executable that creates an object `sample` of `Sample` class and then obtains the value of parameter `value_` in this object.
    *See main.cpp

3) What do we call functions of a class?
    *class functions are given the name member functions.

4) What access specifiers are used?
    *The public and private class specifiers are used to limit access to certain class members. In this case the member variable is denoted private so it is only accessable by internal class members.

5) What do we call variable `value_` in the `Sample` class?
    *'value_' is known as a member variable within the sample class.