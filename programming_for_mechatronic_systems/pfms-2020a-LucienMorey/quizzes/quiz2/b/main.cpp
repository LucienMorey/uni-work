// We need to include the declaration of our new car class in order to use it.
#include "car.h"

#include <iostream>
#include <thread>  // std::this_thread::sleep_for
#include <vector>

int main(void)
{
  // Some specifications provided here, though you can use any of your own

  // Mercedes C180 Compressor.
  // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

  // Bugatti Veyron Super Sport.
  // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg

  //! TODO - TASK 2 : instatiate two objects of type `Car` with different specifications and determine's their top
  //! speed.
  // Create a Bugatti and Mercedes object
  Car mercedes("Mercedes", "C180 Compressor", 1.45, 1.77, 143, 0.29, 1200);
  Car bugatti("Bugatti", "Veyron Super Sport", 1.19, 2.0, 1200, 0.35, 2200);

  std::cout << "mercedes top speed is " << mercedes.calculateTopSpeed() << " m/s" << std::endl;
  std::cout << "bugatti top speed is " << bugatti.calculateTopSpeed() << "m/s" << std::endl;

  bool still_racing = true;
  bool bugatti_reached_top_speed = false;
  bool mercedes_reached_top_speed = false;

  bool mercedes_first = true;
  bool bugatti_first = true;

  // Slow down the thread for 100 miscroseconds
  std::this_thread::sleep_for(std::chrono::microseconds(200));

  std::chrono::steady_clock::time_point mercedes_stop;
  std::chrono::steady_clock::time_point bugatti_stop;
  //! Race until races ends
  while (still_racing)
  {
    //! TODO - TASK 4 : Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and
    //! determines the time difference between the two vehicles reaching zero

    //! Accelerate cars to top speed
    //! Decelearate after reachig top speed to zero
    if (!mercedes_reached_top_speed)
    {
      // std::cout<<"pong" <<std::endl;
      mercedes.accelerate();
    }
    else
    {
      mercedes.decelerate();
    }

    if (!bugatti_reached_top_speed)
    {
      bugatti.accelerate();
    }
    else
    {
      bugatti.decelerate();
    }

      std::cout<< "bugatti speed " << bugatti.getCurrentSpeed() <<std::endl;
      std::cout<< "mercedes speed " << mercedes.getCurrentSpeed() <<std::endl;
    //! Print when each car reach top speed
    if (mercedes.getCurrentSpeed() >= mercedes.calculateTopSpeed())
    {
      mercedes_reached_top_speed = true;
      std::cout << "meredes reached top speed" << std::endl;
    }

    if (bugatti.getCurrentSpeed() >= bugatti.calculateTopSpeed())
    {
      bugatti_reached_top_speed = true;
      std::cout << "bugatti reached top speed" << std::endl;
    }

    // record when cars reach 0
    if ((bugatti.stationary()) && (bugatti_reached_top_speed) && (bugatti_first))
    {
      bugatti_stop = std::chrono::steady_clock::now();
      bugatti_first = false;
      std::cout << "bugatti stopped" << std::endl;
    }

    if ((mercedes.stationary()) && (mercedes_reached_top_speed) && (mercedes_first))
    {
      mercedes_stop = std::chrono::steady_clock::now();
      mercedes_first = false;
      std::cout << "mercedes stopped" << std::endl;
    }

    if ((mercedes.stationary()) && (bugatti.stationary()))
    {
      still_racing = false;
    }

    //! NOTE: Keep the below sleep for the thread of 50 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }

  std::chrono::duration<double> time_dif = bugatti_stop - mercedes_stop;
  std::cout << "the time diference between the cars was " << abs(std::chrono::duration_cast<std::chrono::milliseconds>(time_dif).count()) << " ms" << std::endl;

  return 0;
}
