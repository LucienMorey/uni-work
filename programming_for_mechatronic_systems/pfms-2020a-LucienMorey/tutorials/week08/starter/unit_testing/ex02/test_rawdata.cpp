#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "../laser.h"
#include "../rangerfusion.h"
#include "../ranger.h"
#include "../sonar.h"
#include "../cell.h"

using namespace std;

//==================== HELPER FUNCTIONS ====================//
void testLimits(vector<double> stream, double min, double max) {
  for(auto measurement: stream) {
//    std::cout << measurement << ",";
    ASSERT_TRUE(measurement <= max);
    ASSERT_TRUE(measurement >= min);
  }
//  std::cout << std::endl;
}
//==================== HELPER FUNCTIONS ====================//

//==================== UNIT TEST START ====================//

//Test that data measurements are within limits
TEST (DataDistanceTest, Laser) {
  Laser l;

  //We arbitarily test the data 100 times as its randomly generated
  for(int i = 0; i < 100; i++) {
    testLimits(l.generateData(), 0.2, 8.0);
  }
}

TEST (DataDistanceTest, Sonar) {
  Sonar s;

  //We arbitarily test the data 100 times as its randomly generated
  for(int i = 0; i < 100; i++) {
    testLimits(s.generateData(), 0.2, 16.0);
  }
}


//Test that multiple sensors produce a larger rawDataArray
TEST (MultipleSensorsTest, LaserAndSonarSimple) {
  //Sensors:
  //  - (1) Sonar {fov=60, angular_res=20, offset=0}
  //  - (2) Laser {fov=180, angular_res=30, offset=0}
  //  
  //
  // Produces two vectors, first with 7 measurements & second with 1.
  
  Sonar s;
  s.setOffset(0);

  Laser l;
  l.setAngularResolution(30);
  l.setOffset(0);

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &l, &s };
  RangerFusion rf;
  rf.setRangers(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();


  auto rawdata = rf.getRawRangeData();

  ASSERT_EQ(2, rawdata.size());
  ASSERT_EQ(7, rawdata[0].size());
  ASSERT_EQ(1, rawdata[1].size());
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
