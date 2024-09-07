#include <iostream>
#include <thread>
#include "radar.h"
#include <vector>
#include <chrono>

using namespace std;

int main (void){

  //! TASK 1:
  //! Instantiate radar object
  Radar radar; //default setting max disatance 80m and scanning time A

  //! Display the default max distance
  cout << "Default max distance is " << radar.getMaxDistance() << "m\n" << endl;

  //! TASK 2
  //! Create a thread tied to the spawn member function of Radar [radar.h](./a/dep/radar.h).
  thread t1(&Radar::spawn, &radar);

  //! Create a while loop in the main runs 50 times and displays the return value of getData member funcion.
  std::vector<double> data = radar.getData();
    //for 80 max distance scanning time A
    int i = 0;
    while (i < 50) {
      auto t1 = std::chrono::high_resolution_clock::now();
      radar.getData();
      auto t2 = std::chrono::high_resolution_clock::now();

      auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

      cout << data[i] << endl;
      cout << "Scanning time: " << duration << " microseconds" << endl;
      i++;
    }

  //! TASK 3: We were not provided a rate for the getData
  //! Using the chrono library and 50 sucsessive calls to getData in the while loop
  //! of your main you have already developed, compute the refresh rate (running rate)
  //! of getData (This will tell us the scanning time of the sensor).
  //See above in while loop for task 3

  //! TASK 4: The scanning time is dependent on the MaxDistance.
  //! Add to your main code that sets the other supported MaxDistance and another
  //! while loop that queries getData another 50 times.
  //! Using the chrono library and these 50 sucsessive calls to getData in the while loop,
  //! compute the refresh rate (running rate) of getData (This will tell us the scanning time
  //! of the sensor in the other supported configuration).

    radar.setMaxDistance(160.0);
    cout << "\n\nNew max distance is " << radar.getMaxDistance() << "m\n" << endl;

    //std::vector<double> data = radar.getData();
      //for 160 max distance scanning time B
      int n = 0;
      while (n < 50) {
          //cout << n << endl;
        auto t3 = std::chrono::high_resolution_clock::now();
        radar.getData();
        auto t4 = std::chrono::high_resolution_clock::now();

        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();

        cout << data[n] << endl;
        cout << "Scanning time: " << duration2 << " microseconds" << endl;
        n++;
      }

  t1.join();
  return 0;
}

