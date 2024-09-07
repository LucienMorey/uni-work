Quiz 5
======

Part A
------
You have been provided a library that implement a Radar sensor. The header file describes the public functions available and settings [radar.h](./a/dep/radar.h)

1) TASK: Create an instance of radar, obtain and display the default max Distance.
 - Done

2) TASK: Create a thread tied to the spawn member function of Radar [radar.h](./a/dep/radar.h). Create a while loop in the main runs 50 times and displays the return value of getData member funcion.
 - Done

3) TASK: We were not provided a rate for the getData, using the chrono library and 50 sucsessive calls to getData in the while loop of your main you have already developed, compute the refresh rate (running rate) of getData (This will tell us the scanning time of the sensor).
 - Done

4) TASK: The scanning time is dependent on the MaxDistance, add to your main code that sets the other supported MaxDistance and another while loop that queries getData another 50 times. Using the chrono library and these 50 sucsessive calls to getData in the while loop, compute the refresh rate (running rate) of getData (This will tell us the scanning time of the sensor in the other supported configuration).
 - Done

5) QUESTION: We have been provided the Radar as a library, did we need to implement functions defined in radar.h (YES/NO)?
 -NO (we didn't need to define the member functions ourselves in a .cpp file)

Part B
------
1) TASK: Look at the code in [folder b](./b), and you will notice that a [library](./b/complex.h) for performing operations on complex numbers has been written. Compile and run the test [executable](./b/test_complex.cpp). The unit test `Divide` will fail. Fix the static method `divide()` in [complex.cpp](./b/complex.cpp) and verify that the test passes after re-compilation.
 - Done

2) TASK: Write separate unit tests in [test_complex.cpp](./b/test_complex.cpp) for each function within [complex.cpp](./b/complex.cpp).
 - Done

3) TASK: Make sure that you can compile and run the executable. Your tests should pick up a bug in the [complex library](./b/complex.cpp). Fix the bug and make sure all tests pass.
 - Neg sign needed in multiply() member function
 - ((a.re * b.re) - (a.im * b.im)), ((a.re * b.im) + (a.im * b.re))

4) QUESTION: What are some of the benefits of unit testing?
 - MAKE THE PROCESS AGILE, FIND SOFTWARE BUGS EARLY, PROVIDES DOCUMENTATION, simplifies debugging process, Refactoring (change the code once and ensure everything else is working), reduces costs by finding bugs early
 
5) QUESTION: Why would a single unit test, testing all the functions in the complex library, be generally considered bad practice?
 - One behavior tested per function means that if a test fails, you know exactly why it failed, and can zero in on the specific problem area. If you have multiple behaviors tested in a single function, a failure in a "later" test may be due to an unreported failure in an earlier test causing bad state.
 - One behavior tested per function means that if that behavior ever needs to be redefined, you only have to worry about the tests specific to that behavior, and not worry about other, unrelated tests.
 - Basically when a test function performs only one test it is much easier to identify which case failed. If you isolate the tests then one test failing doesn't affect the execution of the other tests.
