#ifndef _CAR_FOLLOWING_MODIFIED_IDM_
#define _CAR_FOLLOWING_MODIFIED_IDM_
#pragma once
#include<cmath>
#include "../../Object/Vehicle/Vehicle.h"

class CarFollowingModifiedIDM
{
public:
	double LOW_SPEED_ZERO = 0.0;

	double CarFolloingMantainDistance;

	CarFollowingModifiedIDM();
	CarFollowingModifiedIDM(double aCarFolloingMaintainingDistance);
	~CarFollowingModifiedIDM();
	double FreeDriving(double aFollowingSpeed, Vehicle AgentVehicle);

	double AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle);

	double AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle, double& aHeadwayDistance);
};
#endif // !_CAR_FOLLOWING_MODIFIED_IDM_

