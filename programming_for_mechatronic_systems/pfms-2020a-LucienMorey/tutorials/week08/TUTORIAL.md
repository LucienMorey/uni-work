
Week 8 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

We will use Google's C++ unit testing framework, [googletest]. A good starting point for using googletest is to read the [primer] document. Also see the provided [unit_testing_examples] in the starter directory.


Unit Testing : Testing your assignment 2 library - Ex01
-------------------------

Using code form A2, follow instructions from [A2_UNIT_TEST_INSTRUCTIONS]
Run the unit tests against these files, do they work? 
What do you need to modify to get your code to work.

**Outside of class, contemplate how you could test your Assignment 2 fusion in more detail**

**BONUS** Unit Testing : Unit testing for your Assignment 2 - Ex02
We provide an example of how you could unit test your own code, on an example of Assignment 2


OpenCV : Samples 
-------------------
 
We have provided two complete examples [access_pixels] and [display_image].

The [display_image] example loads an image supplied as a parameter on the command line. Download an image, maybe [grumpy_cat] and give it a go.

Example two [access_pixels] demonstrates how to access individual pixels of an image. The key here is line `unsigned char &pixel = image.at<unsigned char>(i, j)`, what does the `unsigned char` mean with respect to `image`? 


OpenCV : Writing code to search along a line in an image
-------------------

We develop a very basic ray tracing example, search between two points in an image for presence of a bluw pixel along the line joining these two points.
Finish the code, to detect if a line between two points collides with a blue pixel using a [line_iterator] for searching.

[access_pixels]: ./starter/opencv/access_pixels
[display_image]: ./starter/opencv/display_image
[drawing_functions]: ./starter/opencv/drawing_functions
[gumpy_cat]: https://www.google.com/search?tbm=isch&as_q=grumpy_cat&tbs=isz:lt,islt:4mp,sur:fmc
[line_iterator]: https://docs.opencv.org/3.1.0/dc/dd2/classcv_1_1LineIterator.html
[A2_UNIT_TEST_INSTRUCTIONS]: ./starter/unit_testing/ex02/README.md 

