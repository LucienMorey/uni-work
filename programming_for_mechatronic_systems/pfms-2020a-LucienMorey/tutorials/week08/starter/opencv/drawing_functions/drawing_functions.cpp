#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>

using namespace cv;
using namespace std;

// Generate a random cv::Point within an image of given dimensions
cv::Point randomPoint(RNG &rng, int width, int height) {
    // Very important that rng is passed by reference
    // check what happens if it is not
    int x = rng.uniform(0, width);
    int y = rng.uniform(0, height);
    return Point(x, y);
}

int main(int argc, char** argv )
{
    // Image dimensions and number of circles
    int width = 600;
    int height = 400;
    int num_circles = 20; // I used 20 to increase collision chances

    // Define some colours (BGR colour space)
    Vec3b white(255,255,255),
          red(0, 0, 255),
          blue(255, 0 , 0);

    // Use OpenCV rand num generator, seeded from system clock
    RNG rng(std::chrono::system_clock::now().time_since_epoch().count());

    // Create an 8-bit 3-channel image of white pixels
    Mat image(height, width, CV_8UC3, white);

    // Draw random blue circles, filled
    for (int i = 0; i < num_circles; i++) {
        int radius = rng.uniform(5, 20);
        Point centre = randomPoint(rng, width, height);
        circle(image, centre, radius, blue, FILLED);
    }

    // Generate random line, and create iterator
    Point pt1 = randomPoint(rng, width, height);
    Point pt2 = randomPoint(rng, width, height);
    LineIterator line_pt(image, pt1, pt2);

    bool did_collide = false;
    // Search the line for blue pixels
    // and while you're there colour them red

    //TODO
    // Insert code that searcher along the line through an image 
    // And check if there is a collision (hitting a blue pixel



    // Print message depending on did_collide
    cout << (did_collide ? "Collision!" : "No Collision") << endl;
    
    // Show the image  
    namedWindow("Image", WINDOW_AUTOSIZE );
    imshow("Image", image);

    // Save the file with the appropriate name depending on did_collide
    string filename = string(did_collide ? "yes" : "no") + ".png";
    imwrite( filename, image);

    // Handle OpenCV gui events, don't worry too much about this
    waitKey(0);

    return 0;
}
