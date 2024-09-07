// We need to include the declaration of our new car class in order to use it.
#include "car.h"

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include<vector>
#include <cmath>

int main (void) {

    //Some specifications provided here, though you can use any of your own

    //Mercedes C180 Compressor.
    // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

    //Bugatti Veyron Super Sport.
    // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg

    //!TODO - TASK 2 : instatiate two objects of type `Car` with different specifications and determine's their top speed.
    // Create a Bugatti and Mercedes object
    Car mercedes("Mercedes", "C180 Compressor", 1.45, 1.77, 143, 0.29, 1200);
    std::cout << "Mercedes Top Speed: " << mercedes.calculateTopSpeed() << std::endl;
    Car bugatti("Bugatti", "Veyron Super Sport", 1.19, 2.0, 1200, 0.35, 2200);
    std::cout << "Bugatti Top Speed: " << bugatti.calculateTopSpeed() << std::endl;

    bool still_racing = true;

    //Slow down the thread for 100 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(200));

    // Struct to associate the car model with testing data (in this case, 
    // model means "simplified representation", not "unique name for this
    // style").
    struct CarTestData_t{
        Car carModel;
        bool hasReachedTopSpeed;
        bool hasFinishedTest;
        double stopTime;
    };

    // Create vector of cars with test data
    // This uses bracketed initialisation, which we need to do, because
    // CarTestData doesn't have a default constructor, because Car doesn't have
    // a default constructor! (The only viable one is the one used above).
    std::vector<struct CarTestData_t> carTestData {
        {mercedes, false, false, 0},
        {bugatti, false, false, 0}
    };
    std::cout << "Starting the race!" << std::endl;
    double current_time = 0;
    //! Race until races ends
    while (still_racing){
        //!TODO - TASK 4 : Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero

        //! Accelerate cars to top speed
        //! Decelearate after reachig top speed to zero
        //! Print when each car reach top speed
        //! Print the car details of fisrt car to reach speed of zero
        //! Print the current speed of all other cars 
        
        still_racing = false; // If no car sets this to true, then we're done.
        for (auto &carData : carTestData) {
            if (!carData.hasReachedTopSpeed) {
                // Still speeding up
                still_racing = true;
                carData.carModel.accelerate();
                if (carData.carModel.getCurrentSpeed() >= carData.carModel.calculateTopSpeed()) {
                    // Has reached top speed
                    carData.hasReachedTopSpeed = true;
                    std::cout << carData.carModel.getMake() << " has reached top speed." << std::endl;
                }
            } else if (!carData.hasFinishedTest) {
                // Still Slowing down
                still_racing = true;
                carData.carModel.decelerate();
                if (carData.carModel.getCurrentSpeed() <= 0) {
                    // Has stopped completely
                    carData.hasFinishedTest = true;
                    std::cout << carData.carModel.getMake() << "has stopped (DEBUG)." << std::endl;
                    carData.stopTime = current_time;
                }
            }
        }

        //! NOTE: Keep the below sleep for the thread of 50 miscroseconds
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        current_time += 0.00005;
        std::cout << "Current Time: " << current_time << "\r";

    }
    // Print race time differences
    std::cout << "Difference in stopping time between " << carTestData[0].carModel.getMake()
        << " and " << carTestData[1].carModel.getMake() << std::endl;
    double diff = carTestData[0].stopTime - carTestData[1].stopTime;
    std::cout << diff << " seconds." << std::endl;
    return 0;
}
