#ifndef _CAR_FOLLOWING_IDM_
#define _CAR_FOLLOWING_IDM_
#pragma once
#include<cmath>
#include "../../Object/Vehicle/Vehicle.h"

class CarFollowingIDM
{
public:
	double LOW_SPEED_ZERO = 0.0;

	double CarFolloingMantainDistance;

public:
	CarFollowingIDM();

	CarFollowingIDM(double aCarFolloingMaintainingDistance);

	~CarFollowingIDM();

	double FreeDriving(double aFollowingSpeed, Vehicle AgentVehicle);

	double FreeDrivingForStringStability(double aFollowingSpeed, Vehicle AgentVehicle, double aTimeStep);

	double AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle);

	double AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle, double& aHeadwayDistance);

	double AgentBasedCarFollowingForStringStability(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed,double aAccelerationLastStep, Vehicle AgentVehicle, double aTimeStep, double& aHeadwayDistance,double& aAcceleration);
	
	double AgentBasedCarFollowingForStringStability(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, double aAccelerationLastStep, Vehicle AgentVehicle, double aTimeStep, double& aHeadwayDistance, double& aAcceleration,double aTimeHeadway_TH);

};
#endif // !_CAR_FOLLOWING_IDM_

