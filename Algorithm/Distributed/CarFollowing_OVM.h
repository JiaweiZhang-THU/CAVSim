#pragma once
#define _USE_MATH_DEFINES 
#include<cmath>
#include "../../Object/Vehicle/Vehicle.h"

class CarFollowing_OVM
{
public:
	double LOW_SPEED_ZERO = 0.0;
public:
	CarFollowing_OVM();
	~CarFollowing_OVM();
	double FreeDriving(double aFollowingSpeed, Vehicle AgentVehicle);
	double AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle);
};

