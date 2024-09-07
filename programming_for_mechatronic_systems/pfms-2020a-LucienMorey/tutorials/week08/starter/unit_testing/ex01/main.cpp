#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <time.h>

#include "rangerinterface.h"
#include "ranger.h"
#include "laser.h"
#include "radar.h"

using namespace std;

void printRawData(std::vector<RangerInterface*>rangers)
{

    for (unsigned int i = 0; i <rangers.size() ; i++){
      std::vector<double> data;
      data = rangers.at(i)->generateData();
        for (unsigned int j = 0; j < data.size(); j++){
              std::cout << data.at(j) << ' ';
        }
        std::cout << std::endl;
    }
}

int main()
{
    Laser laser1;

    Radar radar1;
    Radar radar2;

    int  radar1Off, radar2Off, laserAng;
    cout<< "what would you like your radar offset to be? (offtset *enter*)"<< endl;
    cin >> radar1Off;
    cout << "initialising radar with offset = "<< radar1Off <<endl;


    cout<< "what would you like your radar2 offset to be? (offtset *enter*)"<< endl;
    cin >> radar2Off;
    cout << "initialising radar with offset = "<< radar2Off <<endl;

    cout<< "what would you like your laser angular resolution  to be? (angular resolution *enter*)"<< endl;
    cin >> laserAng;

    cout << "initialising laser with angular resolution = "<< laserAng << "\n" << endl;

    radar1.setOffset(radar1Off);
    radar2.setOffset(radar2Off);
    laser1.setAngularResolution(laserAng);


    std::vector<RangerInterface*>rangers;
    rangers.push_back(&radar1);
    rangers.push_back(&radar2);
    rangers.push_back(&laser1);

    std::cout<<"To receive data press ENTER, to terminate press CTRL+C"<<std::endl;
    std::cin.get();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


    while(1){
      std::cout<<"raw data:"<<std::endl;

      printRawData(rangers);
      std::cout<<"--------------------------------------------------------------------------------"<<std::endl;

  }
  return 0;
}
