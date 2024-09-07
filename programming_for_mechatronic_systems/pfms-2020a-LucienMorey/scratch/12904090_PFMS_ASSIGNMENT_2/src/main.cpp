#include "laser.h"
#include "sonar.h"
#include "rangerfusion.h"

#include <iostream>

int main()
{
    Laser laser;
    Sonar sonar1;
    Sonar sonar2;
    RangerFusion fusion;

    std::cout << "the properties of laser1 are: " 
    << "\nsensor model: " << laser.getSensorModel()
    << "\nmin range: " << laser.getMinRange()
    << "\nmax range: " << laser.getMaxRange()
    << "\nangle resolution: " << laser.getAngularResolution()
    << "\nfield of view: " << laser.getFieldOfView()
    << "\n"
    << std::endl;

    std::cout << "the properties of sonar1 are: " 
    << "\nsensor model: " << sonar1.getSensorModel()
    << "\nmin range: " << sonar1.getMinRange()
    << "\nmax range: " << sonar1.getMaxRange()
    << "\nangle resolution: " << sonar1.getAngularResolution()
    << "\nfield of view: " << sonar1.getFieldOfView()
    << "\n"
    << std::endl;

    std::cout << "the properties of sonar2 are: " 
    << "\nsensor model: " << sonar2.getSensorModel()
    << "\nmin range: " << sonar2.getMinRange()
    << "\nmax range: " << sonar2.getMaxRange()
    << "\nangle resolution: " << sonar2.getAngularResolution()
    << "\nfield of view: " << sonar2.getFieldOfView()
    << "\n"
    << std::endl;

    std::vector<RangerInterface*> rangers;
    std::vector<Cell*> cells;
    cells.push_back(new Cell);

    double mid_x, mid_y;
    cells.at(0)->getCentre(mid_x,mid_y);

    std::cout << "properties of the cell are:"
    << "\nmid x at: " << mid_x
    << "\nmid y at: " << mid_y
    << "\nside length" << cells.at(0)->getSide()
    << "\n"
    << std::endl;

    rangers.push_back(&laser);
    fusion.setRangers(rangers);
    fusion.setCells(cells);
    while(1){
        fusion.grabAndFuseData();
        std::cout << "ping";
        std::cin.ignore();
        while (std::cin.get() != '\n')
    ;
    }
    return 0;
}