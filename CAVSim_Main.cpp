#include <iostream>
#include <list>
#include<math.h>
#include<ctime>
#include <pthread.h>
#include <functional>
#include <future>

// #include "Simulator/simulatorSingleIntersection.h"
// #include "Simulator/SimulatorMerging.h"
#include "Simulator/SimulatorStraightLaneForStringStability.h"

int main()
{
    cout << "\n\t CAVSim: a traffic simulator for connected and automated vehicles (CAVs) \n\n";

    //----------------------------------- Cooperative Driving at Signal-free Intersections -----------------------------------
    /*Simulator1 SimulatorOfIntersection;
    SimulatorOfIntersection.SimulatorId = rand() % 10000 + 10000;
    SimulatorOfIntersection.Run();*/

    //----------------------------------- Cooperative Driving at On-Ramps ----------------------------------- 
    /*SimulatorMerging SimulatorOfOnRamp;
    SimulatorOfOnRamp.SimulatorId = rand() % 10000 + 20000;
    SimulatorOfOnRamp.Run();*/

    //----------------------------------- String Stability Testing of CAV Platoon ----------------------------------- 
    SimulatorStraightLaneForStringStability SimulatorOfStraightLaneForStringStability;
    SimulatorOfStraightLaneForStringStability.SimulatorId = rand() % 10000 + 30000;
    SimulatorOfStraightLaneForStringStability.Run();

    return 0;
}