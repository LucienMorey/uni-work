Quiz 2
======

Part A
------

1) TASK: Modify the base class `Shape` file [shape.h](./a/shape.h) such that function getArea() is defined in `Shape`, and the child classes are required to implement this function.  
HINT: Polymorphism and the concept of virtual.

See `shape.h`.

2) TASK: Create a Square and Traingle, and store both of them in a vector of `Shape` pointers

See `main.cpp`.

3) TASK: Add `Cicrle` as a child class of `Shape`.
HINT: Use the Circle class developed in Week 03

See `circle.h`, `circle.cpp`. Copied from my week 3 tutorial work.

4) TASK: Create a function that loops through shapes and display their area.
HINT: Think of the function signature.

See `main.cpp`.

5) TASK: Write a program that allows the user to specify number of circles and `max_radius`. Create the shapes with random lengths to be capped to `max_length`.

See `main.cpp`.

Part B
------

1) TASK: Modify the file [car.h](./a/car.h) so that it inherits behaviour from the base class, [controllerinterface](./a/controllerinterface.h).

See `car.h`.

2) TASK: Instatiate two objects of type `Car` with different specifications and determine's their top speed.

See `main.cpp`.

3) QUESTION: When instantiating an object of the car class, which constructor(s) are caled and in what order.

Since the `Car` class inherits from `ControllerInterface`, the constructor for
`ControllerInterface` is called first, and then the constructor for `Car` is
called.

4) TASK: Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero.

See implementation in `main.cpp`.

5) QUESTION: Class [controllerinterface](./a/controllerinterface.h) is a special class,  and has special member functions, their syntax is replacing a definition with `=0`. What is the name of these special member functions and special class.

These member functions, and the class itself, are called "purely virtual", because by definition, they do not have an implementation for their functions. If all the member
functions of a class are purely virtual, then that class itself is also purely virtual.
In order for an object of this class to be instantiated, it MUST be inherited into a subclass. 
