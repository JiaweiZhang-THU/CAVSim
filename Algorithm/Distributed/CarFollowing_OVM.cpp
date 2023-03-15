#include "CarFollowing_OVM.h"
#include "CarFollowingOVMParameters.h"

CarFollowing_OVM::CarFollowing_OVM()
{
}

CarFollowing_OVM::~CarFollowing_OVM()
{
}

double CarFollowing_OVM::FreeDriving(double aFollowingSpeed, Vehicle AgentVehicle)
{
	double UpdatedFollowingSpeed = 0;
	double acc = 0;
	acc = OVM_PARAMETERS_alpha * (AgentVehicle.MaxStraightSpeed - aFollowingSpeed) + OVM_PARAMETERS_beta * (110/3.6 - aFollowingSpeed);
	acc = fmin(AgentVehicle.MaxStraightAccel, acc);

	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + acc * paraCarFolloingTimeStep_OVM);

	return UpdatedFollowingSpeed;
}

double CarFollowing_OVM::AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle)
{
	double UpdatedFollowingSpeed = 0;
	double acc = 0;
	double OptimalVelocity = 0;

	if (aHeadDistance >= OVM_PARAMETERS_hgo)
	{
		OptimalVelocity = AgentVehicle.MaxStraightSpeed;
	}
	else if (aHeadDistance > OVM_PARAMETERS_hst) 
	{
		OptimalVelocity = AgentVehicle.MaxStraightSpeed / 2 * (1 - cos(M_PI * (aHeadDistance - OVM_PARAMETERS_hst) / (OVM_PARAMETERS_hgo - OVM_PARAMETERS_hst)));
	}
	acc = OVM_PARAMETERS_alpha * (OptimalVelocity - aFollowingSpeed) + OVM_PARAMETERS_beta * (aHeadSpeed - aFollowingSpeed);
	acc = fmin(AgentVehicle.MaxStraightAccel, acc);

	UpdatedFollowingSpeed = fmax(0, fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + acc * paraCarFolloingTimeStep_OVM));

	return UpdatedFollowingSpeed;
}