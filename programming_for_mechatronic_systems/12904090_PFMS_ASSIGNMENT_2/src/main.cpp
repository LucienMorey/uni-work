#include "laser.h"
#include "sonar.h"
#include "rangerfusion.h"

#include <iostream>
#include <random>
#include <chrono>

// typedef to assist with namespacing issues
typedef std::chrono::steady_clock Time;

// constant scanning time to help break up sensor readings
const double SCAN_TIME = 1.0;

int main()
{
  // create sensors
  Laser laser;
  Sonar sonar1;
  Sonar sonar2;
  RangerFusion fusion;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // output the fixed paramaters of laser as strings in terminal as per task E2
  std::cout << "the properties of laser1 are: "
            << "\nsensor model: " << laser.getSensorModel() << "\nmin range: " << laser.getMinRange() << "m"
            << "\nmax range: " << laser.getMaxRange() << "m";
  // check sensing method enum and then about correct method as string
  if (laser.getSensingMethod() == CONE)
  {
    std::cout << "\nsensing method: Cone";
  }
  else
  {
    std::cout << "\nsensing method: Point";
  }
  // leave space for clarity in terminal
  std::cout << "\n" << std::endl;

  // output the fixed paramaters of sonar 1 as strings in terminal as per task E2
  std::cout << "the properties of sonar1 are: "
            << "\nsensor model: " << sonar1.getSensorModel() << "\nmin range: " << sonar1.getMinRange() << "m"
            << "\nmax range: " << sonar1.getMaxRange() << "m";
  // check sensing method enum and then about correct method as string
  if (sonar1.getSensingMethod() == CONE)
  {
    std::cout << "\nsensing method: Cone";
  }
  else
  {
    std::cout << "\nsensing method: Point";
  }
  // leave space for clarity in terminal
  std::cout << "\n" << std::endl;

  // output the fixed paramaters of sonar 2 as strings in terminal as per task E2
  std::cout << "the properties of sonar2 are: "
            << "\nsensor model: " << sonar2.getSensorModel() << "\nmin range: " << sonar2.getMinRange() << "m"
            << "\nmax range: " << sonar2.getMaxRange() << "m";
  // check sensing method enum and then about correct method as string
  if (sonar2.getSensingMethod() == CONE)
  {
    std::cout << "\nsensing method: Cone";
  }
  else
  {
    std::cout << "\nsensing method: Point";
  }
  // leave space for clarity in terminal
  std::cout << "\n" << std::endl;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // configure the angle resolution for all sensors
  unsigned int angle_resolution;
  // configure the laser
  std::cout << "please enter the angle resolution for the laser: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are 10 , 30. if no acceptable paramater is used it will default to 10"
            << std::endl;
  // repeat input while a non unsigned integer value is input
  while (!(std::cin >> angle_resolution))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set the entered value or use default
  laser.setAngularResolution(angle_resolution);

  // configure sonar1
  std::cout << "please enter the angle resolution for sonar1: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are 20. if no acceptable paramater is used it will default to 20"
            << std::endl;
  // repeat loop while non unsigned integer value is input
  while (!(std::cin >> angle_resolution))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set the entered value or use default
  sonar1.setAngularResolution(angle_resolution);

  // configure sonar 2
  std::cout << "please enter the angle resolution for sonar2: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are 20. if no acceptable paramater is used it will default to 20"
            << std::endl;
  // repeat input while non unsigned integer value is input
  while (!(std::cin >> angle_resolution))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set the entered value or use default
  sonar2.setAngularResolution(angle_resolution);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // set angle offset of sensors
  int angle_offset;
  // set offset for laser
  std::cout << "please enter the angle offset for the laser: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are signed ints." << std::endl;

  // repeat while angle offset is non integer value
  while (!(std::cin >> angle_offset))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set angle offset value
  laser.setOffset(angle_offset);

  // set angle offset of sonar1
  std::cout << "please enter the angle offset for sonar 1: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are signed ints." << std::endl;

  // repeat while angle offset is non integer value
  while (!(std::cin >> angle_offset))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set angle offset value
  sonar1.setOffset(angle_offset);

  // set angle offset of sonar2
  std::cout << "please enter the angle offset for sonar2: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are signed ints." << std::endl;

  // repeat while angle offset is non integer value
  while (!(std::cin >> angle_offset))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set angle offset value
  sonar2.setOffset(angle_offset);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // configure the FOV for all sensors
  unsigned int field_of_view;
  // configure the laser
  std::cout << "please enter the FOV for the laser: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are 180. if no acceptable paramater is used it will default to 180"
            << std::endl;
  // repeat input while a non unsigned integer value is input
  while (!(std::cin >> field_of_view))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set the entered value or use default
  laser.setFieldOfView(field_of_view);

  // configure sonar1
  std::cout << "please enter the FOV for sonar1: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are 20. if no acceptable paramater is used it will default to 20"
            << std::endl;
  // repeat loop while non unsigned integer value is input
  while (!(std::cin >> field_of_view))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set the entered value or use default
  sonar1.setFieldOfView(field_of_view);

  // configure sonar 2
  std::cout << "please enter the FOV for sonar2: " << std::endl;
  std::cout << "Note that acceptable paramater(s) are 20. if no acceptable paramater is used it will default to 20"
            << std::endl;
  // repeat input while non unsigned integer value is input
  while (!(std::cin >> field_of_view))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }
  // set the entered value or use default
  sonar2.setFieldOfView(field_of_view);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<RangerInterface*> rangers;

  // create vector of ranger interfaces for rangerfusion
  rangers.push_back(&laser);
  rangers.push_back(&sonar1);
  rangers.push_back(&sonar2);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // create cells ffor use in fusion
  std::vector<Cell*> cells;

  unsigned int num_of_cells;
  // get number of cells desired by user
  std::cout << "please enter the number of elements you wish to generate: " << std::endl;
  // repeat while input value is non unsigned int
  while (!(std::cin >> num_of_cells))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }

  // obtain maximum range from ranger interface vector
  double max_range = 0;
  for (auto ranger : rangers)
  {
    if (ranger->getMaxRange() > max_range)
    {
      max_range = ranger->getMaxRange();
    }
  }
  // create random number generator for generating cell centre position
  int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  // distribution between -max range and max for range 0 to max sensing range for domain
  std::uniform_real_distribution<double> distX(-max_range, max_range);
  std::uniform_real_distribution<double> distY(0.0, max_range);

  // generate specified number of cells
  for (unsigned int i = 0; i < num_of_cells; i++)
  {
    // add them to vector of cells
    cells.push_back(new Cell);
    // configure cells to have randim centre within sensor range
    cells.at(i)->setCentre(distX(generator), distY(generator));
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // display cell centre points
  double mid_x, mid_y;
  std::cout << "Cell Centre points are at:\n";
  int cell_counter = 0;
  for (auto cell : cells)
  {
    cell->getCentre(mid_x, mid_y);
    std::cout << "Cell No. " << cell_counter + 1 << "has centre at " << mid_x << ", " << mid_y << "" << std::endl;
    cell_counter++;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // point ranger fusion to cells and ranger vectors
  fusion.setRangers(rangers);
  fusion.setCells(cells);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // create timer object to help breakut scans
  Time timer;
  Time::time_point last_scan_time;
  std::chrono::duration<double> scan_time;

  // counter for how many readings have taken place
  int iteration_counter = 0;

  while (1)
  {
    // generate new data drom sensors and fuse data to determine cell state
    fusion.grabAndFuseData();

    // record time of last scan
    last_scan_time = timer.now();

    std::cout << "iteration " << iteration_counter << ": ";
    // print out state of cells
    for (auto cell : cells)
    {
      if (cell->getState() == OCCUPIED)
      {
        std::cout << "occupied, ";
      }
      else if (cell->getState() == FREE)
      {
        std::cout << "free, ";
      }
      else
      {
        std::cout << "unknown, ";
      }
    }
    std::cout << std::endl;

    // update the amount of loops that have taken place
    iteration_counter++;

    // calculate time difference between scan time and now
    // block until ime difference is greater than 1 second
    scan_time = timer.now() - last_scan_time;
    while (scan_time.count() < SCAN_TIME)
    {
      scan_time = timer.now() - last_scan_time;
    }
  }

  for (auto cell : cells)
  {
    delete cell;
  }
  return 0;
}