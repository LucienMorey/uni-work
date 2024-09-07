#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermocksonar.h"


//TEST(GrabBeforeFuse,Segfault)
//{
//
//  Sonar s;
//  s.setOffset(0);
//
//  Laser l;
//  l.setAngularResolution(30);
//  l.setOffset(0);
//
//  std::vector<Cell*> cells;
//  cells.push_back(new Cell());
//  cells.at(0)->setCentre(0,0.5);
//
//  std::vector<RangerInterface *> sensors = { &l, &s };
//  RangerFusion rf;
//  rf.setRangers(sensors);
//  rf.setCells(cells);
//
//  ASSERT_EXIT((rf.grabAndFuseData(),exit(0)),::testing::KilledBySignal(SIGSEGV),".*");
 
//}


TEST(GrabBeforeFuse,Pass)
{

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

  ASSERT_EXIT((rf.grabAndFuseData(),exit(0)),::testing::ExitedWithCode(0),".*");

}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
