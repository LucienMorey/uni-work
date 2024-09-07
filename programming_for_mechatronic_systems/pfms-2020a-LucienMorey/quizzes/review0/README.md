Quiz 0
======

Part A
------
1) Attempt to compile the code in [folder a](./a). Why does the code fail to compile?

Because it has two functions that do the same thing therefore it is trying to redefine and cannot compile. 

2) Fix the compilation error. Why does a segmentation fault (segfault) occur?

Segmentation fault can occur when you are trying to access restricted data. In this case, it is because you are trying to access data that exceeds the bounds of the array. 

3) Modify the code so it will run

4) Create a function that prints only the elements of the array that are larger than : mean + one standard deviation

5) Create a function that assigns elements of array x to a vector named `vec` (HINT: decide on the correct type)

Part B
------
1) Look at the code in [folder b](./b). Implement the methods of the Sample class in the [sample.cpp](./b/sample.cpp) file based on the definition provided in [sample.h](./b/sample.h)

2) Make an executable that creates an object `sample` of `Sample` class and then obtains the value of parameter `value_` in this object.

3) What do we call functions of a class?

Member functions.

4) What access specifiers are used?

Public: Members accessible from outside the class.

Private: Members cannot be accessed from outside the class. 

5) What do we call variable `value_` in the `Sample` class?

Private.
