#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

std::mutex mu;
int shared_int (0);

void increment () {
   for (int i=0; i<10000000; ++i) {
      std::lock_guard<std::mutex> guard(mu);
      // mu.lock();
      shared_int++;
      //mu.unlock();
   }
};

void incrementNum (int &num) {
   for (int i=0; i<10000000; ++i) {
      std::lock_guard<std::mutex> guard(mu);
      num++;
   }
}

int main ()
{
   //std::thread th1(increment);
   //std::thread th2(increment);
   std::thread th1(incrementNum, std::ref(shared_int));
   std::thread th2(incrementNum, std::ref(shared_int));
   th1.join();
   th2.join();
   std::cout << "Final value: " << shared_int << std::endl;
   return 0;
}
