Quiz 2
======

Part A
------

1) TASK: Modify the base class `Shape` file [shape.h](./a/shape.h) such that function getArea() is defined in `Shape`, and the child classes are required to implement this function.  
HINT: Polymorphism and the concept of virtual.

2) TASK: Create a Square and Traingle, and store both of them in a vector of `Shape` pointers

3) TASK: Add `Cicrle` as a child class of `Shape`.
HINT: Use the Circle class developed in Week 03

4) TASK: Create a function that loops through shapes and display their area.
HINT: Think of the function signature.

5) TASK: Write a program that allows the user to specify number of circles and `max_radius`. Create the shapes with random lengths to be capped to `max_length`.


Part B
------

1) TASK: Modify the file [car.h](./a/car.h) so that it inherits behaviour from the base class, [controllerinterface](./a/controllerinterface.h).

2) TASK: Instatiate two objects of type `Car` with different specifications and determine's their top speed.

3) QUESTION: When instantiating an object of the car class, which constructor(s) are caled and in what order.
    * When instantiating a car object, both the car constructor and the base class controllerinterface constructor are called. The default controllerInterface constrictor is callled first. Then the specified Car constructor is called.

4) TASK: Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero.

5) QUESTION: Class [controllerinterface](./a/controllerinterface.h) is a special class,  and has special member functions, their syntax is replacing a definition with `=0`. What is the name of these special member functions and special class.
    * These member funtions are pure virtual. This makes it impossible to initialise an object of that class and froces any subclasses to implement those methods.


