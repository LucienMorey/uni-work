#include <iostream>
#include <thread>
#include <chrono>
#include "radar.h"

int main(void)
{
  //! TASK 1:
  //! Instantiate radar object
  //! Display the default max distance
  std::shared_ptr<Radar> radar(new Radar());
  std::cout << "radar max distance " << radar->getMaxDistance() << std::endl;

  //! TASK 2
  //! Create a thread tied to the spawn member function of Radar [radar.h](./a/dep/radar.h).
  std::thread radar_thread(&Radar::spawn, radar);
  //! Create a while loop in the main runs 50 times and displays the return value of getData member funcion.

  //! TASK 3: We were not provided a rate for the getData
  //! Using the chrono library and 50 sucsessive calls to getData in the while loop
  //! of your main you have already developed, compute the refresh rate (running rate)
  //! of getData (T
  std::chrono::system_clock::time_point t1;
  std::chrono::system_clock::time_point t2;

  int counter = 0;
  std::vector<double> radar_data;
  while (counter < 50)
  {
    // take time before data gen
    t1 = std::chrono::system_clock::now();
    // gen data
    radar_data = radar->getData();
    // take time after data gen
    t2 = std::chrono::system_clock::now();

    // display data gen time duration
    std::cout << "scan time was " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds" << std::endl;
    // display data
    std::cout << "radar data ";
    for (auto data : radar_data)
    {
      std::cout << data << ", ";
    }
    std::cout << std::endl;
    // increment loop counter
    counter++;
  }
  //! TASK 4: The scanning time is dependent on the MaxDistance.
  //! Add to your main code that sets the other supported MaxDistance and another
  //! while loop that queries getData another 50 times.
  //! Using the chrono library and these 50 sucsessive calls to getData in the while loop,
  //! compute the refresh rate (running rate) of getData (This will tell us the scanning time
  //! of the sensor in the other supported configuration).

  // set new max range
  radar->setMaxDistance(160);
  // add whitespace to differentiate between ranges then display new range
  std::cout << "\n \n \n \n"
            << "radar max distance " << radar->getMaxDistance() << std::endl;
  // reset counter
  counter = 0;
  while (counter < 50)
  {
    // take time point before data gen
    t1 = std::chrono::system_clock::now();
    // gen data
    radar_data = radar->getData();
    // take data point after data gen
    t2 = std::chrono::system_clock::now();

    // display data gen time
    std::cout << "scan time was " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds" << std::endl;

    // display radar data
    std::cout << "radar data ";
    for (auto data : radar_data)
    {
      std::cout << data << ", ";
    }
    std::cout << std::endl;

    // increment loop counter
    counter++;
  }

  // wait for thread to join
  radar_thread.join();

  return 0;
}
