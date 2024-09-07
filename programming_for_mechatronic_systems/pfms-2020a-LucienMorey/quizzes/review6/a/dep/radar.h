#ifndef RADAR_H      // An 'include guard' to prevent double declaration of any identifiers in this library
#define RADAR_H

#include <string>
#include <vector>
#include <random>

class Radar{
    public:

    Radar();                                        // Constructor for Radar, default setting max disatance 80m and scanning time A
    std::vector<double> getData(void);	            // return a vector of targets, blocking call function which is tied to the scanning time
    double getMaxDistance(void);                    // get Max Distance for Radar
    bool setMaxDistance(double maxDistance);        // set Maximum Distance for Radar (valid setting 80 and 160), which result in scanning time A and B.
    void spawn();                                   // start the thread that generates data

private:
    void generateData(void);
    std::vector<double> data_;                       //targets as a vector
    double scanningTime_;
    const unsigned int numTarget_ = 20;
    double maxDistance_ ;
    const double minDistane_ = 0.1;                   //min distance for Radar
    std::mt19937 *generator_;
    std::uniform_real_distribution<double> *value_;

};

#endif // SENSOR_H
