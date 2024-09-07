Quiz 3
======

Part A
------

1) TASK: In `main()` create a number of circles as specified in [main.cpp](./a/main.cpp) and push them back to vector of Shape*
    *See main.cpp

2) TASK: In `main()` create an object of [Line Class](./a/line.cpp), using the two points method. Use the parameters that have been provided from the user, `x1 y1` and `x2 y2`
    *See main.cpp

3) TASK: Implement the missing function(s) in [Analysis Class](./a/analysis.cpp).
HINT: Analysis inherits from AnalysisInterface which is an abstract class
    *See analysis.cpp

4) TASK: Create an object of class Analysis and pass the shapes and line to analysis using the approprate member functions.
    *See main.cpp

5) TASK: In `main()` compute the area of all shapes that intersect the line using the return value of [intersectsLine] member function.
    *See main.cpp

Part B
--------------------

Modify the file [sum_doubles.cpp](./b/sum_doubles.cpp) such that it

1) TASK: Creates a vector of doubles with 4 random values (between 0 and 100) 
    *See sum_doubles.cpp

2) TASK: Adds a random value to the front of the vector
    *See sum_doubles.cpp. Note added to beginning and end of the vector here because of conflicting instruction here and in sum_doubles.cpp TODO

3) TASK: Replaces the 3rd value of the vector with another random number
    *See sum_doubles.cpp

4) TASK: Print out the numbers using a range-based for loop with the auto keyword 
    *See sum_doubles.cpp

5) TASK: Computes the sum via [accumulate function](https://en.cppreference.com/w/cpp/algorithm/accumulate) using an iterator and print the sum to console
    *See sum_doubles.cpp

