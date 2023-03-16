#pragma once
#include "../../Object/Vehicle/Vehicle.h"
#include "../Centralized/DrivingPlan.h"

class EnergyOptimalControl2
{
public:
	EnergyOptimalControl2();
	~EnergyOptimalControl2();

public:
	double TimeStep;

	double delta;

	int ControlPeriod;

	void Run(DrivingPlan& NewPlan, const Vehicle& aVehicle, const double t_assign);

protected:
	BPointCoordinate Position2Location(const BPointCoordinate& oldLocation, const double newPosition, const double Angle);

	bool RunWithCollisionDetection(DrivingPlan& NewPlan, const Vehicle& aVehicle, double& t_assign, double& Distance, const double v_0, const double v_f, const bool detect);
};

