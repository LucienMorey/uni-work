Quiz 4
======

Part A
------

1) TASK: Create a function that accepts a deque and modifies it by adding user specified numbers of elements, each element is form a gaussian distribution (mean: 8, std dev 4) [deque_vector_ops.cpp](./a/deque_vector_ops.h)

* see populateDeque fucntion in deque_vector_ops.cpp

2) TASK: Create a functions that prints the chosen container
* created vector functions in deque_vector_ops.cpp

3) We are now tasked to design a function to rearrange elements of out container (vector or deque) by bubble sort operation,refer for pseudo code  (https://en.wikipedia.org/wiki/Bubble_sort).
QUESTION: Which STL container is more suitable for the Bubble sort and why?
* In this case a vector is more suitable for the bubble sort application. there is no need to access the data from either end of the contianer and the location of the stored elements in memory isirrelevaent since values are swapped. As a consequence a performance will be seen.

4) TASK: Create a function that accept the chosen container and rearranges elements by bubble sort operation
* see bubbleSortVector in deque_vector_ops.cpp

5) TASK: In the main call sorting function and print the container after operation (re-use function developed in step 2)
* see main in deque_vector_ops.cpp

Part C
-------

Consider the code in [data_race.cpp](./b/data_race.cpp)  

1) QUESTION: How many threads are running in parallel (disregarding the main)?
* There are 2 values running aside from the main there are two threads running: th1,th2.

2) QUESTION: What value would you expect to see printed?
* I would expect to see 20000000 as the threads would both add on top of each other

3) QUESTION: What is the specific problem with this code, causing it to fail?
* There is no data security in the threads. The threads are likely trying to write at the same time, causing an error. these increments are likely lost or causing unexpected results.

4) TASK: Implement one approach to fix the problem and outline merits of the solution.
* I implemented a mutex lock. This approach is good because it will block if it cannot obtain the lock and ensures all increments to occur.

5) TASK: Instead of having `int shared_int` as a global variable, change the code to pass this variable to function `increment ()`
* increment function now takes a struct type that includes a mutex and int for counting as per dat_race.cpp

