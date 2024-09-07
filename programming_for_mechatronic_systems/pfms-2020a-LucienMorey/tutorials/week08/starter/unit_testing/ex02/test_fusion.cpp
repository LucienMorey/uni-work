#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "../rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermockradar.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SingleLaserFusionTest, LaserFree) {
// Laser
// res: 30.0000
// off: 0.0000
// ranges: 7.48515,5.49413,6.11037,5.99643,3.25937,5.31273,1.53526,
//
// Cell parameters
// centre [x,y]=[0.0000,2.9982]
//
// Fusion FREE (LASER Through);
//

  RangerMockLaser r1(180, 30, 0, {7.48515,5.49413,6.11037,5.99643,3.25937,5.31273,1.53526});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,2.9982);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1};
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::FREE};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cells.at(0)->getState(),expected);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (SingleSonarFusionTest, SonarFree) {

//Sonar
//res: 20.0000
//off: 30.0000
//range: 7.24026 

//Cell parameters
//centre [x,y]=[-2.4763,3.4018]

// Fusion Free

  RangerMockSonar r2(20, 20, 30.0, {7.24026});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-2.4763,3.4018);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::FREE};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cells.at(0)->getState(),expected);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (DefaultDataFusionTest, LaserIntersectsSonarsMiss) {

//Laser
//res: 30.0000
//off: 0.0000
//ranges: 3.31830,2.22699,6.44053,3.56503,7.30305,1.61841,2.25766,
//
//Sonar 1
//res: 20.0000
//off: 30.0000
//ranges: 2.49952 
//
//Sonar 2
//res: 20.0000
//off: -30.0000
//ranges: 2.34988 
//
//Cell parameters
//centre [x,y]=[-3.6515,6.3246]


  RangerMockLaser l(180, 30, 0, {3.31830,2.22699,6.44053,3.56503,7.30305,1.61841,2.25766});
  RangerMockSonar s1(20, 20, 30, {2.49952});
  RangerMockSonar s2(20, 20, -30, {2.34988});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-3.6515,6.3246);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &l, &s1, &s2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::OCCUPIED};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cells.at(0)->getState(),expected);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
