#pragma once

#include<map>
#include "../../Object/Road/BItem.h"
#include "../../Object/Vehicle/Vehicle.h"

class ArrivalTime
{
public:
	list<int> PasssingOrder;

	double LargestArrivalTimeInConflictSubzones[20][20];

	ArrivalTime();
	ArrivalTime(double aLargestArrivalTimeInConflictSubzones[20][20], int aLaneNum);

	~ArrivalTime();

	double PassingOrderToTrajectoryInterPretaton(list<int> aPassingOrder, map<int, Vehicle>& SimuVehicleDict, double aSimuTimeNow);

	double CalculMinimumArrivalTimeBangBangControl(Vehicle aVehicle, double aSimuTimeNow);

	double CalculMinimumArrivalTimeAtOnRamp(Vehicle aVehicle, double aSimuTimeNow);

	void CalculMinimumArrivalTimeToOtherSubconflcitzones(Vehicle& aVehicle);

	void AdjustTimeAssignAccordingToTheConstantVelocityInTheConflictZone(Vehicle& aVehicle);
};
