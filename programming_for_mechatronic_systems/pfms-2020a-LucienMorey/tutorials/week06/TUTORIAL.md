Week 6 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

Critical Sections - Ex01
-----------------------------------------

Consider a case where we want to process a stream of incoming data as fast as possible using multithreading

Create a program which:
* Fills a queue with one million non-zero numbers which sum to zero
* Spawns two threads in parallel which each repeatedly pop the front of the queue and add it to their subtotal until the the queue is empty
* Combines the two subtotals to compute the sum of all numbers from the queue

Hint: _To ensure synchronization use std::mutex and std::lock_guard_

Multi-Thread Access Data - Ex02
-------------------------

Three threads should access same struct

Struct contains:
* string name
* vector of doubles

Thread 1
* adds random number (from uniform distribution 0-100) to the vector of doubles 
Thread 2
* removes numbers less than 20 and greater than 80
Thread 3
* Keeps size of vector to max 20 elements, removes oldest element

Questions:
* How best to protect data?
* What should a efficient implementation do?

Multi-Thread Access Data - Ex03
-------------------------

Instead of a strcut, use a class. Apart from the member variables, the class should also contain member functions that handle the data.
The main, should have no knowledge of mutexes, as they can be embeded in the class itself.

