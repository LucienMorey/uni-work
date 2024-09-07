#include "image_processing.h"
#include <image_transport/image_transport.h>

ImageProcessing::ImageProcessing():
  debug_(false){
}

ImageProcessing::ImageProcessing(cv::Mat image, bool debug):
  image_(image),debug_(debug){
}

void ImageProcessing::setDebug(bool debug){
  debug_=debug;
}

void ImageProcessing::setImage(cv::Mat image){
  image.copyTo(image_);//This will copy the image into our member variable (image) courtesy of OpenCV
}

bool ImageProcessing::checkConnectivity(cv::Point origin, cv::Point destination)
{

  if(image_.empty()){
    std::cout << "No image supplied yet" << std::endl;
    return false;
  }

  // Following visualisation code (in the #ifdef - #endif) is only enabled for the tutorial
  // To understand the inner working.
  // When delivering the code as a library it should be removed (comment out #define)
  if(debug_){
    // Create an 8-bit 3-channel image from input image, so we can draw on it
    cv::Mat color_image;
    cv::cvtColor(image_, color_image, cv::COLOR_GRAY2BGR);
    // Draw the origin in Green (the colors are BGR so G is 255 and other two channels zero)
    cv::circle(color_image,origin,1,CLR_ORIGIN,2);
    // Draw destination in Red
    cv::circle(color_image,destination,1,CLR_GOAL,2);
    // Let's put some text, just for additional understanding
    cv::putText(color_image, "green - origin ,red - destination", cv::Point(0,10), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(255,0,0));
    // Make a window called debug, which can be adjusted in size
    cv::namedWindow("debug",cv::WINDOW_NORMAL);
    // Draw the color image on the windows called "debug"
    cv::imshow("debug",color_image);
    // wiat for a maximum of 5000ms for a user to click on window, then close window
    cv::waitKey(5000);
    cv::destroyWindow("debug");
  }

  //! Create a Line Iterator  between origib abd destination
  cv::LineIterator lineIterator(image_, origin, destination);

  //! @todo Ex01 : Insert code to search the line for non-white pixel
  //! If there is a non-white pixel it means the robot would not be able to
  //! Travel over the line origin and destination
  //!
  //! code needed here ---
  //! Look at Week 08 tutorial examples - drawing function

  // Search the line of white pixels
  for (int i = 0; i < lineIterator.count; i++, lineIterator++) {
      uchar *pixel = (uchar*)*lineIterator;
      if (*pixel < 255) {
          return false;
      }
  }

  return true;

}
