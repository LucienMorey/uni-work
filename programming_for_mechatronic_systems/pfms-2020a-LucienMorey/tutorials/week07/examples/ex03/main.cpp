#include <iostream>
#include <thread>
#include <vector>

#include "radar.h"
#include "dataprocessing.h"

int main (void){

  //! We create a vector of pointers to Radar
  std::vector<Radar*> radars;
  //! Push pack 3 radars
  radars.push_back(new Radar);
  radars.push_back(new Radar);
  radars.push_back(new Radar);

  //! Vector of threads
  std::vector<std::thread> threads;

  //! Pushback each radar
  for (auto radar : radars){
    threads.push_back(std::thread(&Radar::start,radar));
  }

  //! Created a pointer to data processing
  std::shared_ptr<DataProcessing> dataProcessingPtr(new DataProcessing());
  //! Pass the radars
  dataProcessingPtr->setRadars(radars);
  //! Add to list of threads
  threads.push_back(std::thread(&DataProcessing::findClosestReading,dataProcessingPtr));

  //Join threads
  for(auto & t: threads){
    t.join();
  }

  return 0;
}

