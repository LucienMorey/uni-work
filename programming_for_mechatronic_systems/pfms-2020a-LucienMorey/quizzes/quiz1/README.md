Quiz 1
======

Part A
------

1) QUESTION Attempt to compile the code in [folder a](./a). Why does it fail? (list the reasons)
    * The person class file was not included in the main. this meant the reference to create a person object failed
    * The attempt to access the member variables of alice fails because they are listed in the class as private

2) TASK Fix the issue so that it compiles.
    *Included person.h
    *Modified code to use setters for Alice

3) TASK Make the code more robust, with respect to "sane" values of age.
    *Altered setAge function to return false and set age to 0 if a negative age or an age greater than 120 is parsed.

4) TASK Create a `crowrd` using a vector container of people, populate it with 3 people.
    *Created vector crowd and used the pushback function to populate it.

5) TASK Create a function that greets the oldest member of the `crowd`.
    *Created the crowdGreeting function in main.cpp.

6) TASK Implement a safe guard to make the initialisation of `person` objects easier? (HINT: What special member function is missing in Person?)
    *Implemented default constructor that inits default name and age.

Part B
------

1) TASK Modify the file rectangle [rectangle](./b/rectangle.h) so it inherits from the base class of shape is [shape](./b/shape.h)
    *altered rectangle.h so the rectangle class inherits from the shape class.

2) TASK Correct the missing access specifiers of base class [shape](./b/shape.h)
    *Added in the public, protected and private access specifiers for the appropriate class members.

3) TASK Modify the main [main.cpp](./b/main.cpp) so that it creates a rectangle of size width=5 and height =3.5
    *A Recangle object is constucted with the default constructor then the width and height are set with the setHeightWidthMethod.

4) QUESTION If you create a `Rectangle`, which constructor is called, and what does it initialise?
    *A rectangle constructor is called initially. This Constructor initialises height and width to the default value of 0.0. Then sets the description of the shape to square and implicitly calls the Shape constructor.
