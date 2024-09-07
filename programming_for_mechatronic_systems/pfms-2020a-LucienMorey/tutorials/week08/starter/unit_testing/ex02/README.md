## Usage

Drop this folder into the area where your A2 code exists. Compilation expects certain source files to exist in the directory one-level up.

Run the following command from within the directory:
```
mkdir build; cd build; cmake ..; make; ./rawTests; ./fusionTests
```

## Raw TESTS

There are 2 test cases: 
DataDistanceTest (2 tests, Laser and Sonar)
MultipleSensorsTest (2 tests)

## Fusion Tests

There are 3 test cases: 

SingleLaserFusionTest: Laser intersects cell (State::FREE) 

<img src="pics/SingleLaserFusionTest.jpg" alt="Laser intersects cell" width="400px"/>

SingleSonarFusionTest: Sonar covers cells (State::FREE)

<img src="pics/SingleSonarFusionTest.jpg" alt="Sonar covers cell" width="400px"/>

DefaultDataFusionTest: laser return in cell, both sonars miss the cell (State::OCCUPIED)

<img src="pics/DefaultDataFusionTest.jpg" alt="Sonars do not cover cell, laser return in cell" width="400px"/>

