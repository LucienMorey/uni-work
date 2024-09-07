#ifndef DISPLAY_H
#define DISPLAY_H

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <random>   // Includes the random number generator

#include "car.h"

const unsigned int MAP_SIZE       =200;
const unsigned int MAP_CENTRE     =100;
const unsigned int RADIUS         = 70;

class DisplayRace
{
public:
  DisplayRace();

  void updateDisplay(std::vector<Car> cars);

private:
  cv::Mat track_;

};

#endif // DISPLAY_H
