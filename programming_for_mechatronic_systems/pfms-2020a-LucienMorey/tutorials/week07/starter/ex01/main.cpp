#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting
#include <condition_variable>

using namespace std;

// The function generates samples
void generateSamples(vector<double> &data, mutex &numMutex, condition_variable &cv) {

  //Setup and seed our random normal distribution generator
  std::default_random_engine generator(std::chrono::duration_cast
                                       <std::chrono::nanoseconds>
                                       (std::chrono::system_clock::now().time_since_epoch()).count());
  std::normal_distribution<double> distribution(6.0, 5.0); //mean of 6m and a stdev of 5m

  while (true) {

        // This delay is included to improve the emulate some other process of generating the data
        // by the sensor which could be at a specific rate
        std::this_thread::sleep_for (std::chrono::milliseconds(1000));


        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        numMutex.lock();

        cout << "sample gen" << endl;
        // We only access num while the mutex is locked
        double sample = distribution(generator);
        data.push_back(sample);

        numMutex.unlock();
        cv.notify_one();

    }
}

// This function consumes the samples
void processSamples(vector<double> &data, mutex &numMutex, condition_variable &cv) {
    while (true) {
        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        unique_lock<mutex> locker(numMutex);
        cv.wait(locker, [&](){return data.size() >= 2;});
        double sum = 0;
        int num_samples = 0;
        for (auto data_p : data){
            sum = sum + data_p;
            num_samples++;
        }

        double mean = sum/num_samples;
        auto closest_to_mean = data.begin();;
        
        for (auto data_p = data.begin(); data_p<data.end(); ++data_p){
            if(abs((*(data_p)-mean)) < abs((*(closest_to_mean)-mean))){
                closest_to_mean = data_p;
            }
        }
        cout << "mean is" << mean << "/n";
        cout << "data before delete" << "\n";
        for(auto data_p : data){
            cout << data_p << ", ";
        }
        cout << endl;
        data.erase(closest_to_mean);
        cout << "data after delete" << "\n";
        for(auto data_p : data){
            cout << data_p << ", ";
        }
        cout << endl;
        double sample = data.front();
        data.pop_back();
        numMutex.unlock();
        // We now have a sample
        cout <<  "sample is:" << sample << endl;
          
    }
}

int main ()
{
    vector<double> data;
    // We will use this mutex to synchonise access to num
    mutex numMutex;

    condition_variable cv;

    // Create the threads
    thread inc_thread(generateSamples,ref(data),ref(numMutex), ref(cv));
    thread print_thread(processSamples,ref(data),ref(numMutex), ref(cv));

    // Wait for the threads to finish (they wont)
    inc_thread.join();
    print_thread.join();

    return 0;
}



