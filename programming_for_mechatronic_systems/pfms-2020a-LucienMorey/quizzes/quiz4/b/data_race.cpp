#include <iostream>
#include <thread>
#include <vector>
#include <mutex>


struct counter_w_lock {
   int counter = 0;
   std::mutex mtx;
}counter; 

void increment (counter_w_lock& counter) {
   for (int i=0; i<10000000; ++i) {
      counter.mtx.lock();
      counter.counter++;
      counter.mtx.unlock();
   }
}

int main ()
{
    std::thread th1(increment, std::ref(counter));
    std::thread th2(increment, std::ref(counter));
    th1.join();
    th2.join();
    std::cout << "Final value: " << counter.counter << std::endl;
    return 0;
}
