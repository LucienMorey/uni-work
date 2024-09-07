Week 1 Tutorial Questions
=========================
We will cover the below concepts in the first session of the semester:
* Linux filesystem
* Bash operations
* cmake 
* compiling (out of source build)
* git (version control)
* qtcreator (IDE)

Thereafter we examine
* pointers
* functions
* vectors

Ex01 - Files on Linux
--------------------
* Create a directory (mkdir)
* Create a text file in it (use gedit and nano)
* Make a copy of the file and modify it (cp)
* Delete the original


Ex02 - Using GIT
--------------------
* clone your classroom repo (you should have done this in pre-work)
* create a week01 folder in scratch
* create a text note in that folder
* add/commit/push to git repo

Ex03 - Compiling cmake projects (command line only)
---------------------
* copy hello world project from tutorials/week01/starter to scratch/week01/
* create build directory
* run cmake
* run make
* run executable
* modify cout text to use their name "Hello <YOUR_NAME>"
* rebuild and run
* commit modified project (without build products)

Ex04 - Compiling cmake projects (qtcreator)
---------------------
* create build directory
* start qtcreator ``qtcreator &``
* rebuild and run

Hint:
* Running on command line enables input

Ex05 - Functions 
---------
* Create a function that accepts a double value as a parameter and
* Returns a bool value if the double is greater than zero and the square value instead of initial passed value.

Question:
* What is a function signature?
A function Signature is instantiated by function parameter type, order and the amount parsed with a function decleration. It is used to handle overload resolution within the compiler.

Ex06 - Functions 
---------
* Create an additional function that accepts a double value as a parameter and
* Returns bool value if the double is greater than zero, the square value, the cube value and the passed value incremented by one

Questions:
* Can this function be called the same name as the existing function created in Ex05?

The function cannot have the same name in this situation because the only difference between the two functions is the return type. In c++ the return type is not a part of the function signature.

Ex07 - Struct
------
* Create a structure called `Sensor` than contains
* A variable for the number of samples `num_samples`
* An array of samples `double data[]`
* Create a sensor from the struct
* Populate data with 5 elements, each data[i] =i. How to code end of loop?
* Create a function that prints samples 
* Can you initialise a sensor of two elements in one line of code?

Questions:
* What is the problem with using an array here?

Arrays in c++ are not flexible. This data array should be the length of the number of samples but that that isnt possible.

* What are the issues with using a struct? (Enter concept of Classes)

The issue with using a struct here is that public access to the number of samples is allowed. Ideally that value would be private so that  outside members couldnt change the amount of data samples within the stucture.
    
Ex08 - Vector
------
* Create a vector of integers called  `data` of length 10 elements 
* Create a loop to populate elements of `data` (each element [i] =i))
* Loop to print elements of vector
Questions:
* How to create a vector of floats?

When declaring the variable use a float tag instead of an int tag. see code for example.

* How could we pass the length to the executable instead of hard coding 10?

When creating the vector it is possible to use an overloaded construtor and parse length as one of the arguements. It is also possible to use the member function resize() to change the vectors length after declaring it.


Ex09 - Vector of Vectors (Container of Data)
------
* Create a vector of vectors containing integers called  `data` 
* Create a loop to populate elements of `data` such that is is a matrix of 4x4 elements (each element row =i))
* Loop to print elements of `data`
Questions:
* What else could you store in this container?

It would be possible to store most ther types of data inthis container after typecasting them to be ints. I would probably only consider storing chars or unsigned types of the same length

