#include "laser.h"
#include <iostream>
#include <iomanip>

int main()
{
  // set precision of values in cout to help keep things readable
  std::cout.precision(3);

  // read in scan time from user to be set in sensor
  double scan_time;
  std::cout << "What scan time would you like to use? (0.02 or 0.06)" << std::endl;
  std::cout << "A value of 0.02 will begin a scan with angle resolution of 30 degrees" << std::endl;
  std::cout << "A value of 0.06 will begin a scan with angle resolution of 10 degrees" << std::endl;
  // repeat if an invalid output is obtained
  while (!(std::cin >> scan_time))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter a decimal value and try again: " << std::endl;
  }

  // create laser object
  Laser laser;

  // set user input scan time
  // alert user if they input an invalid scan time
  if (!laser.setScanningTime(scan_time))
  {
    std::cout << "invalid scan time entered. system will default to 0.02s scan time and 30 degree resolution"
              << std::endl;
  }

  // display all fixed parameters
  std::cout << "Sensor model is: " << laser.getModel() << std::endl;
  std::cout << "Sensor output is: " << laser.getOutputType() << std::endl;
  std::cout << "Sensor field of view is: " << laser.getFieldOfView() << std::endl;
  std::cout << "Sensor max range is: " << laser.getMaxRange() << std::endl;
  std::cout << "Sensor min range is: " << laser.getMinRange() << std::endl;

  // block until user ready for data
  std::cout << "press enter to begin data output" << std::endl;
  std::cin.ignore();
  while (std::cin.get() != '\n')
    ;

  // start 5 second clock for first query stage
  std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
  std::chrono::duration<double> timer = std::chrono::system_clock::now() - t1;

  // scan and output data
  while (timer.count() < 5)
  {
    std::cout << "Sample No. " << laser.getSampleCount() << " contains: ";
    for (auto x : laser.getData())
    {
      std::cout << std::setw(5) << x << ", ";
    }
    std::cout << std::endl;
    timer = std::chrono::system_clock::now() - t1;
  }

  std::cout << "5 seconds has passed" << std::endl;

  // read in angle res from user to be set
  int angle_res;
  std::cout << "What angle resolution would you like to use? (30 or 10)" << std::endl;
  std::cout << "A value of 30 will begin a scan with scan time of 0.02 seconds" << std::endl;
  std::cout << "A value of 10 will begin a scan with scan time of 0.06 seconds" << std::endl;
  // repeat while a non integer input is read
  while (!(std::cin >> angle_res))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }

  // set user input angle res
  // alert user if they input an invalid value
  if (!(laser.setAngleResolution(angle_res)))
  {
    std::cout << "invalid angle resolution entered. system will not change configuration" << std::endl;
  }

  // block until user ready for data
  std::cout << "press enter to begin data output" << std::endl;
  std::cin.ignore();
  while (std::cin.get() != '\n')
    ;

  // scan and print indefinitely
  while (1)
  {
    std::cout << "Sample No. " << laser.getSampleCount() << " contains: ";
    for (auto x : laser.getData())
    {
      std::cout << std::setw(4) << x << ", ";
    }
    std::cout << std::endl;
  }

  return 0;
}