#include "CarFollowing_Secondorder.h"
#include "CarFollowingSoParameters.h"

CarFollowing_Secondorder::CarFollowing_Secondorder()
{
}

CarFollowing_Secondorder::~CarFollowing_Secondorder()
{
}

double CarFollowing_Secondorder::FreeDriving(double aFollowingSpeed, Vehicle AgentVehicle)
{
	double UpdatedFollowingSpeed = 0;

	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + AgentVehicle.MaxStraightAccel * paraCarFolloingTimeStep_So);

	return UpdatedFollowingSpeed;
}

double CarFollowing_Secondorder::AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle)
{
	double UpdatedFollowingSpeed = 0;

	double acc = 0;

	acc = So_PARAMETERS_kd * (aHeadDistance - So_PARAMETERS_d_des) + So_PARAMETERS_kv * (aHeadSpeed - aFollowingSpeed) + So_PARAMETERS_kv * (aFollowingSpeed - So_PARAMETERS_v_des);
	acc = fmin(AgentVehicle.MaxStraightAccel, acc);

	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + acc * paraCarFolloingTimeStep_So);
	UpdatedFollowingSpeed = fmax(UpdatedFollowingSpeed, 0);

	return UpdatedFollowingSpeed;
}