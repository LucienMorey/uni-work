#include <iostream>
#include <stdlib.h>
#include "radar.h"
#include <vector>
#include <chrono>
#include <thread>


Radar::Radar():
  scanningTime_(100),ready_(false){

  //Generate first random value
  std::random_device rd;
  generator_= new std::mt19937(rd());
  value_ = new std::uniform_real_distribution<double>(0.1,maxDistance_);

  data_.resize(numTarget_);
}

void Radar::start(){
  while(true){
    generateData();
  }

}

void Radar::generateData(){
  //generate random number of targets for each target (N) create Target containing random range and bearing between ^stored values
  // Info on dereferecing pointer https://stackoverflow.com/questions/27081035/why-should-i-dereference-a-pointer-before-calling-the-operator-in-c/27081074#27081074

  //We can now create a wait which will behave like a real sensor if we lock the mutex
  std::unique_lock<std::mutex> lck(mtx_);
  for (unsigned int i=0; i < numTarget_; i++){
    data_.at(i)=value_->operator()(*generator_);
  }
  ready_=true;
  lck.unlock();
  cv_.notify_all();
  std::this_thread::sleep_for (std::chrono::milliseconds(static_cast<int>(scanningTime_)));
}


std::vector<double> Radar::getData(){
  //! We wait for the convar to release us, unless data is already ready
  std::unique_lock<std::mutex> lck(mtx_);
  while (!ready_) cv_.wait(lck);
  std::vector<double> data = data_;
  ready_=false;
  lck.unlock();
  //The below piece of code emulates a real sensor blocking call
  return data;
}


void Radar::setScanningTime(double scanningTime){
  scanningTime_ = scanningTime;
}

double Radar::getScanningTime(void){
  return scanningTime_;
}

double Radar::getMaxDistance(void){
  return maxDistance_;
}
