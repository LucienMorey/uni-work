// We need to include the declaration of car class in order to use it.
#include "car.h"
// We need to include the declaration of display class in order to use it.
#include "display_race.h"

#include <iostream>
#include <thread>  // std::this_thread::sleep_for
#include <vector>

int main(void)
{
  //! @todo
  //! TASK 1
  //! Create 3 cars with follwing specifications
  //!
  //! CONSIDER: We will be using all the cars for a race and need to treat all of them as a collection

  // Mercedes - C180
  // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg
  Car mercedes("Mercedes", "C180", 1.45, 1.77, 143.0, 0.29, 1200.0);

  // Bugatti - Veyron
  // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg
  Car bugatti("Bugatti", "Veyron", 1.19, 2.0, 1200.0, 0.35, 2200.0);

  // Toyota - Yaris_WRsC
  // height = 1.19 m, width = 1.87 m, power P = 420 HP, drag coefficient = 0.30, weight = 1190 kg
  Car toyota("Toyota", "Yaris_WRsC", 1.19, 2.00, 420.0, 0.3, 1190.0);
  // Add the cars to a vector of Cars
  std::vector<Car> cars;

  cars.push_back(mercedes);
  cars.push_back(bugatti);
  cars.push_back(toyota);

  //! @todo
  //! TASK 2
  //! Write a loop that uses the 'Cars' and prints out the make, model and top speed
  //!
  //! CONSIDER: If you have 3 seperate cars you will not be able to loop over them

  for (auto car : cars)
  {
    std::cout << "make: " << car.getMake() << "\n"
              << "model: " << car.getModel() << "\n"
              << "top speed: " << car.getTopSpeed() << "\n"
              << "\n";
  }

  bool still_racing = true;

  //! @todo
  //! TASK 3
  //!
  //! Race the vehicles by:
  //! 1. Accelerating each vehicle until it reachs top speed
  //! 2. When each vehicle reaches top speed deccelerate it
  //! 3. When the first vehicle reaches zero speed then stop the race
  std::vector<bool> cars_reached_top_speed(3, false);
  DisplayRace raceDisplay;  // This creates a OpenCV window to display the race

  //! Race until time lapsed is less than duration or reaching top speed
  while (still_racing)
  {
    // Uncomments the below once you have a vector of Car called cars
    raceDisplay.updateDisplay(cars);
    // std::cout << "ping" << std::endl;
    auto cars_itr = cars.begin();
    int i = 0;

    for (; cars_itr != cars.end(); i++, ++cars_itr)
    {
      if (!cars_reached_top_speed.at(i))
      {
        (*cars_itr).accelerate();
      }
      else
      {
        (*cars_itr).decelerate();
      }

      if ((*cars_itr).getCurrentSpeed() >= (*cars_itr).getTopSpeed())
      {
        cars_reached_top_speed.at(i) = true;
      }

      if (((*cars_itr).getCurrentSpeed() == 0) && (cars_reached_top_speed.at(i)))
      {
        still_racing = false;
      }
    }

    // Slow down the thread for 50 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return 0;
}
