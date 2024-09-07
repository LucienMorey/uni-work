Quiz 1
======

Part A
------

1) QUESTION Attempt to compile the code in [folder a](./a). Why does it fail? (list the reasons)

The code fails to compile because the person.h header file has not been included in the main.cpp. As such, the program doesnt recognise the person class. The next reason the code does not compile is beacuse the variables name_ and age_ are private variables within the class so they cannot be modified outside of the class.

2) TASK Fix the issue so that it compiles.

Done

3) TASK Make the code more robust, with respect to "sane" values of age.

Done. Modified the setAge() function so that the value if age_ is always positive. I also modified the code to check all of the ages and tell the user if any of them are not within this value. I hope thats what this question is asking for.

4) TASK Create a `crowrd` using a vector container of people, populate it with 3 people.

Done

5) TASK Create a function that greets the oldest member of the `crowd`.

Done

6) TASK Implement a safe guard to make the initialisation of `person` objects easier? (HINT: What special member function is missing in Person?)

Done. Added a new constructor to the class so that the persons age and name are specified when the object is created rather than having to add them in separate functions.

Part B
------

1) TASK Modify the file rectangle [rectangle](./b/rectangle.h) so it inherits from the base class of shape is [shape](./b/shape.h)

Done

2) TASK Correct the missing access specifiers of base class [shape](./b/shape.h)

Done

3) TASK Modify the main [main.cpp](./b/main.cpp) so that it creates a rectangle of size width=5 and height =3.5

Done

4) QUESTION If you create a `Rectangle`, which constructor is called, and what does it initialise?

When a rectangle is created, the default constructor is called. It sets the width and height of the rectangle to be 0.0 and also sets the description to be "square".
