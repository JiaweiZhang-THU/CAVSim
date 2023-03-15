#ifndef _DRIVING_PLAN_
#define _DRIVING_PLAN_

#include "../../Object/Vehicle/Vehicle.h"

class DrivingPlan
{
public:
	DrivingPlan();
	~DrivingPlan();
public:
	list<Vehicle> SimuVehOrderList;
	string StrategyType;
	int LeadingVehicleId;
	int MappingVehicleId;
	list<double> LeadDistList;
	int WaitingOrder;

	list<double> AngularSpeedList;
	list<double> SpeedList;
	list<BPointCoordinate> PositionList;
	list<double> PoseAngleList;
	list<double> DistanceList;
private:
	int _count;

public:
	void SetInitInformation(int aStrategyId, int aLeadingCarId, int aMappingCarId);

	void Add(BPointCoordinate aPosition, double aPoseAngle, double aSpeed, double aAngularSpeed);

	void Run(double& aSpeed, double& aAngularSpeed);

	int GetCount();

	DrivingPlan DeepCopy();

	void Clear();
};

#endif // !_DRIVING_PLAN_