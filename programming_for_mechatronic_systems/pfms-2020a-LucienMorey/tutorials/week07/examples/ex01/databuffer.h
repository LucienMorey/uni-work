#ifndef DATABUFFER_H
#define DATABUFFER_H

#include <iostream>
#include <mutex>
#include <vector>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting


class DataBuffer
{
public:
  DataBuffer();



private:
  std::vector<double> data;
  // We will use this mutex to synchonise access to num
  std::mutex numMutex;




};

#endif // DATABUFFER_H
