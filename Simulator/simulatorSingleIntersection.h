#ifndef _SIMULATOR_
#define _SIMULATOR_

#pragma once
#include<list>
#include<math.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>

#include<Windows.h>
#include<io.h>
#include<direct.h>

#include "../Object/Vehicle/Vehicle.h"
#include "../Object/Road/StraightLane.h"
#include "../Object/Road/SingleIntersection.h"
#include "../Algorithm/Distributed/LaneChangingModel.h"
#include "../Algorithm/Distributed/TrajectoryPlanning.h"
#include "../Algorithm/Others/cls_random.h"

#include "../Algorithm/Centralized/DrivingPlan.h"
#include "../Algorithm/Centralized/PlanBasedApproach.h"
#include "../TestingRelated/IntersectionStrategyEvaluation.h"

#include "../Algorithm/Others/Tools.h"

using namespace std;


class Simulator1 {
public:
	double TimeStep;

	int	SimulatorId;

	double TimeIntervalForUpdateTrafficFlowRate;

	double VehileFlowPossionLambda;

	int SimulatingStep;
	double SimulatingStep2Time;

	SingleIntersection SingleIntersectionInstance;

	Vehicle VehicleArray[5000];
	int VehicleNum;
	int VehicleIdIterator;

	map<int, DrivingPlan> VehicleDrivingPlan;

	list<Vehicle> VehicleList;
	list<Vehicle> VehicleOverList; 

	void ClearVehicle();

	void UpdateVehicle(Vehicle aVehicle);

	void AddVehicle(Vehicle aVehicle);

	void RemoveVehicle(Vehicle aVehicle);

	Simulator1();

	double RandomExponential();

	list<int> VehicleFlowGenerator(int aVehicleNum, int aBeginTime);

	list<int> VehicleFlowReadFromFile();

	BNode InitialBNodeForNewVehicle();

	bool CreateNewVehicle(int aNewVehicleId, Vehicle& NewVehicle);

	bool IsLaneChangingPermitted(Vehicle aVehicel, int aLaneChangeAction);

	IntersectionStrategyEvaluation MyIntersectionStrategyEvaluation;

	void UpdateVehicle2MapBlock(Vehicle& aVehicle);

	void SetVehiclePointerOnStraightLaneUnit(Vehicle& aVehicle, StraightLane& aStraightLane);

	void UpdateVehicle2StraightLane(Vehicle& aVehicle);

	void UpdateVehicle2Intersection(Vehicle& aVehicle);

	bool VehicleAreaPositioningUpdate(Vehicle& aVehicle);

	list<Vehicle> PerceptionSurroundingVehicles(Vehicle& aVehicle);

	bool IsInOriginalBlock(Vehicle& aVehicle);

	cls_random MyCls_Random;

	TrajectoryPlanning MyTrajectoryPlanning;

	PlanBasedApproach* MyPlanBasedApproach;

	bool CooperativeDrivingAtIntersection();

	void Run();

	void InitialStatisticalVariable();

	double RadiusOfIntersectionControlZone;

	void IsVehickEnteringIntersectionControlZone(Vehicle& aVehicle);

};


#endif // !_SIMULATIOR_