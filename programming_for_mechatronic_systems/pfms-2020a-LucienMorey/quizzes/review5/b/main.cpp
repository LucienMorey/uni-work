#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>
#include <mutex>

std::mutex mtx;
//Define a new data structure called 'TDataBuffer' and instansiate a global variable
//of this structure called 'sequence'.
struct TDataBuffer {
  std::queue<long> buffer;
} sequence;

void fibonacci(void) {
  static long a = 0;
  static long b = 1;
std::unique_lock<std::mutex>lock(mtx);
  sequence.buffer.push(a);
  while(true) {
    sequence.buffer.push(b);
    b = a + b;
    a = b - a;

    std::this_thread::sleep_for(std::chrono::milliseconds(750));
    lock.unlock();


  }
}

void printToTerminal(void) {
std::unique_lock<std::mutex>lock(mtx);

  while(true) {
    if(sequence.buffer.empty()) {
      std::cout << "<buffer was empty>" << std::endl;
    } else {
      long next = sequence.buffer.front();
      sequence.buffer.pop();

      std::cout << "Next in sequence: " << next << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
        lock.unlock();
  }
}


int main (void) {
  //Create the threads
  std::thread producer(fibonacci);
  std::thread consumer(printToTerminal);

  return 0;
}
