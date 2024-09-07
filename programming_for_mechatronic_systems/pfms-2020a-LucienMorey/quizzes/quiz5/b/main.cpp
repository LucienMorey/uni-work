#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

//Define a new data structure called 'TDataBuffer' and instansiate a global variable
//of this structure called 'sequence'.
struct TDataBuffer {
  std::queue<long> buffer;
  std::condition_variable cv;
  std::mutex mx;
};

void fibonacci(TDataBuffer& sequence) {
  std::unique_lock<std::mutex> locker(sequence.mx);
  static long a = 0;
  static long b = 1;

  sequence.buffer.push(a);
  locker.unlock();
  while(true) {
    //lock sequence while data is generated
    std::unique_lock<std::mutex> locker(sequence.mx);
    //push latest fibonacci number
    sequence.buffer.push(b);
    //calculate next fib number
    b = a + b;
    a = b - a;

    //unlock and alert printing thread
    locker.unlock();
    sequence.cv.notify_one();

    //left delay so that overflow doesnt occur instantly
    std::this_thread::sleep_for(std::chrono::milliseconds(750));
  }
}

void printToTerminal(TDataBuffer& sequence) {
  long next;
  while(true) {
    //lock sequence
    std::unique_lock<std::mutex> locker(sequence.mx);
    //unlock until data is ready
    sequence.cv.wait(locker ,[&](){return !sequence.buffer.empty();});
    //print and remove latest element
    next = sequence.buffer.front();
    sequence.buffer.pop();
    std::cout << "Next in sequence: " << next << std::endl;
    sequence.mx.unlock();
  }
}


int main (void) {
  TDataBuffer sequence;
  //Create the threads
  std::thread producer(fibonacci, std::ref(sequence));
  std::thread consumer(printToTerminal, std::ref(sequence));

  //end threads
  producer.join();
  consumer.join();

  return 0;
}
