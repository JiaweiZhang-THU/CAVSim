#ifndef _SIMULATORMERGING_
#define _SIMULATORMERGING_

#pragma once
#include<list>
#include<math.h>
#include <iostream>
#include <fstream>
#include <io.h>
#include <direct.h>
#include <string>

#include "../Object/Vehicle/Vehicle.h"
#include "../Object/Road/MergingBlock.h"
#include "../Algorithm/Distributed/LaneChangingModel.h"
#include "../Algorithm/Distributed/TrajectoryPlanning.h"
#include "../Algorithm/Others/cls_random.h"
#include "../Algorithm/Centralized/PlanBasedApproach_ramp.h"

#include "../TestingRelated/IntersectionStrategyEvaluation.h"

using namespace std;

class SimulatorMerging
{
public:

	double TimeStep;

	double TimeIntervalForUpdateTrafficFlowRate;

	int LaneNum;

	double VehileFlowPossionLambda;

	int SimulatorId;

	int SimulatingStep;

	double SimulatingStep2Time;

	MergingBlock MergingScenario;

	int AccumulaterdVehicleNum;

	list<Vehicle> VehicleList;

	list<Vehicle> VehicleOverList;

	map<int, DrivingPlan> VehicleDrivingPlan;

	void ClearVehicle();

	void UpdateVehicle(Vehicle aVehicle);

	void AddVehicle(Vehicle aVehicle);

	void RemoveVehicle(Vehicle aVehicle);

	SimulatorMerging();

	double RandomExponential();

	list<int> VehicleFlowGenerator(int aVehicleNum, int aBeginTime);

	list<int> VehicleFlowReadFromFile();

	BNode InitialBNodeForNewVehicle();

	IntersectionStrategyEvaluation MyMergingStrategyEvaluation;

	bool CreateNewVehicle();

	void UpdateVehicle2MapBlock(Vehicle& aVehicle);

	bool IsLaneChangingPermitted(Vehicle aVehicel, int aLaneChangeAction);

	int ThresholdOfVehicleNumInSignalLanUnit;

	void CheckWhetherSimulationNeedToBeReset();

	list<Vehicle> PerceptionSurroundingVehicles(Vehicle aVehicle);

	list<Vehicle> PerceptionSurroundingVehiclesPlus(Vehicle aVehicle);

	cls_random MyCls_Random;

	LaneChangingModel MyLaneChangingModel;

	TrajectoryPlanning MyTrajectoryPlanning;

	double RadiusOfRampControlZone;

	void IsVehickEnteringRampControlZone(Vehicle& aVehicle);

	PlanBasedApproach_ramp* MyPlanBasedApproach;

	bool CooperativeDrivingAtOnRamp();

	int TimeStepIntervalBetweenSnapshot;

	bool IsLoadHLCMModel;

	bool IsForHLCMTest;

	bool IsNoLaneChangeForComparision;

	int IntervalBetweenTwoRlControlledAV;

	void Run();
};

#endif // !_SIMULATORMERGING_
