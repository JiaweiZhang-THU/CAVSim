#pragma once
#include<cmath>
#include "../../Object/Vehicle/Vehicle.h"
class CarFollowing_Secondorder
{
public:
	double LOW_SPEED_ZERO = 0.0;


public:
	CarFollowing_Secondorder();
	~CarFollowing_Secondorder();
	double FreeDriving(double aFollowingSpeed, Vehicle AgentVehicle);
	double AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle);
};

