#ifndef _SIMULATOR_
#define _SIMULATOR_

#pragma once
#include<list>
#include<math.h>
#include <iostream>
#include <fstream>
#include <io.h>
#include <direct.h>
#include <string>

#include "../Object/Vehicle/Vehicle.h"
#include "../Object/Road/StraightLane.h"
#include "../Algorithm/Distributed/LaneChangingModel.h"
#include "../Algorithm/Distributed/TrajectoryPlanning.h"
#include "../Algorithm/Others/cls_random.h"

using namespace std;


class SimulatorStraightLaneForStringStability {
public:
	double TimeStep;

	double TimeIntervalForUpdateTrafficFlowRate;

	int LaneNum;

	double VehileFlowPossionLambdaPerLane;

	int SimulatorId;

	int SimulatingStep;

	double SimulatingStep2Time;

	StraightLane ModularizedStraightLane;

	int AccumulaterdVehicleNum;

	list<Vehicle> VehicleList;

	list<Vehicle> VehicleOverList;

	void ClearVehicle();

	void UpdateVehicle(Vehicle aVehicle);

	void AddVehicle(Vehicle aVehicle);

	void RemoveVehicle(Vehicle aVehicle);

	SimulatorStraightLaneForStringStability();

	double RandomExponential();

	list<int> VehicleFlowGenerator(int aVehicleNum, int aBeginTime);

	BNode InitialBNodeForNewVehicle();

	bool CreateNewVehicle();

	void UpdateVehicle2MapBlock(Vehicle& aVehicle);

	bool IsLaneChangingPermitted(Vehicle aVehicel, int aLaneChangeAction);

	int ThresholdOfVehicleNumInSignalLanUnit;

	void CheckWhetherSimulationNeedToBeReset();

	void InitializeAPlatoonOfVehicles(int aVehicleNum);

	list<Vehicle> PerceptionSurroundingVehicles(Vehicle aVehicle);

	cls_random MyCls_Random;

	LaneChangingModel MyLaneChangingModel;

	TrajectoryPlanning MyTrajectoryPlanning;

	int TimeStepIntervalBetweenSnapshot;

	bool IsNoLaneChangeForComparision;

	int IntervalBetweenTwoRlControlledAV;

	void SpeedDisturbanceForTheLeadingVehicle_TestScenario_Decelerate();

	void SpeedDisturbanceForTheLeadingVehicle_TestScenario_Accelerate();

	double StringStabilityTesting_PacketLossRatio_PLR;

	double StringStabilityTesting_BeaconSendingFrequence_BSF;

	double StringStabilityTesting_MaxAccelerationOfVehicle;

	double StringStabilityTesting_TimeHeadway_TH;

public:
	void Run();
};

#endif // !_SIMULATIOR_