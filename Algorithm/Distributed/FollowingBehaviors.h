#pragma once
#include<math.h>
#include "../../Object/Vehicle/Vehicle.h"
class FollowingBehaviors
{
public:
	FollowingBehaviors();
	~FollowingBehaviors();

	double DelayInTime;
	double MaintainingDistance;
	double DeltaTime;

public:
	double VeryLowSpeed;
	double IsTimeToConsiderMapping;

	double Sigma;

public:
	void AgentBasedFollowingBehavior(double noise, double headDist, double headV, double& currSpeed,Vehicle simuVehicle);

	void PlanBasedFollowingBehaviorWhenMapping(double noise, double headDist, double headV, double& currSpeed,Vehicle simuVehicle,double selfDist);
private:

};
