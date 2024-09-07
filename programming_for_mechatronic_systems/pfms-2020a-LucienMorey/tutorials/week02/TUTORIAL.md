Week 2 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

Classes - Ex01
--------------------
Implement a circle class that has:
* A constructor that sets the radius
* A Method that sets the radius
* A Method that returns the area
* A Method that returns the perimeter

Questions:
* What data should we store in the class
    *The radius of the circle
* How do we set the accessibility of methods (member functions) and member variables?
    *By using the access specifiers: public, private or protected, external entteties are given full access, no access or access if they are subclasses from an object respectively.
Classes - Ex02
--------------------
Write a program that 
* Creates three circles (or radius 1.0, 2.0 and 5.0)
* Computes the area of these circles
* Computes the perimeter of these circles

Questions:
* Is there a way we can store multiple objects of the same class in a container?
    * it is possible to make a vector of circles to store multiple cirlces

Classes - Ex03
------------------
Write a: 
* Function that accepts a vector of circles and returns the combined area of all circles in the vector.
* Program that creates a vector of circles with random radii (the radii can be between 1.0 and 10.0) and uses the function to compute the combined area.

Hints:
[Random Generator](http://www.cplusplus.com/reference/random/uniform_real_distribution/)
* Use a seed for a Random Number Generator 
* Allow user to specify how many random circles are generated

Questions:
* If we used a C style array, what would be the pinch points here, how could we ensure the program does not cause a segmentation fault?
    * Issues stem from the size of a C array. An array must have static sizing, this means that a user wont be able to generate the desired size of an array. To get around this and avoid a segmentation fult from a null pointer, the array must  be greatly oversized to avoid crashing.
* What method in STL allows access to elements of the container?
    * the at() method allows access to the element at that location within a container.
* What is a segmentation fault?
    * A segmentation fault is a memory management fault. In this situation the most likely cause for one would trying to access memory that doesnt belong to the container or a Null pointer from an empty array element.
* What is a exception?
    * an exception is a method of catching undesirable state during a program's life. A possible use might be to catch a run time error and direct the program down a specific logic path to handle it instead of crashing.

Bonus Questions:
* If we wanted to create a circle of random size within Circle class, how should we use the Random Number Generator?
    * It would be possible to create a circle of random size in the class constructor. By seeding the generator and getting a single value it would be posible to set the radius class member to equal this value during construction.





