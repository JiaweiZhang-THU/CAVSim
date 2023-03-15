#include "CarFollowing_Modified_IDM.h"
#include "CarFollowingModifiedIDMParameters.h"

CarFollowingModifiedIDM::CarFollowingModifiedIDM()
{

	CarFollowingModifiedIDM::CarFolloingMantainDistance = Modified_IDM_PARAMETERS_So;
}

CarFollowingModifiedIDM::CarFollowingModifiedIDM(double aCarFolloingMaintainingDistance)
{
	CarFollowingModifiedIDM::CarFolloingMantainDistance = aCarFolloingMaintainingDistance;
}


CarFollowingModifiedIDM::~CarFollowingModifiedIDM()
{
}

double CarFollowingModifiedIDM::FreeDriving(double aFollowingSpeed, Vehicle AgentVehicle)
{
	double UpdatedFollowingSpeed = 0;

	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + AgentVehicle.MaxStraightAccel * paraCarFolloingTimeStep_Modified_IMD);

	return UpdatedFollowingSpeed;

}


double CarFollowingModifiedIDM::AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle)
{

	double local_IDM_PARAMETERS_a = AgentVehicle.MaxStraightAccel;
	double S_star = Modified_IDM_PARAMETERS_So + fmax(0, aFollowingSpeed * Modified_IDM_PARAMETERS_To + aFollowingSpeed * (aFollowingSpeed - aHeadSpeed) / (2 * sqrt(local_IDM_PARAMETERS_a * Modified_IDM_PARAMETERS_b)));

	double U_acc = local_IDM_PARAMETERS_a * fmin(1 - pow(aFollowingSpeed / AgentVehicle.MaxStraightSpeed, Modified_IDM_PARAMETERS_Delta), 1 - pow(S_star / aHeadDistance, 2));


	double UpdatedFollowingSpeed = 0;
	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + U_acc * paraCarFolloingTimeStep_Modified_IMD);
	UpdatedFollowingSpeed = fmax(UpdatedFollowingSpeed, 0);

	return UpdatedFollowingSpeed;

}


double CarFollowingModifiedIDM::AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle, double& aHeadwayDistance)
{
	double local_IDM_PARAMETERS_a = AgentVehicle.MaxStraightAccel;
	double S_star = Modified_IDM_PARAMETERS_So + fmax(0, aFollowingSpeed * Modified_IDM_PARAMETERS_To + aFollowingSpeed * (aFollowingSpeed - aHeadSpeed) / (2 * sqrt(local_IDM_PARAMETERS_a * Modified_IDM_PARAMETERS_b)));
	aHeadwayDistance = S_star;

	double U_acc = local_IDM_PARAMETERS_a * (1 - pow(aFollowingSpeed / AgentVehicle.MaxStraightSpeed, Modified_IDM_PARAMETERS_Delta) - pow(S_star / aHeadDistance, 2));


	double UpdatedFollowingSpeed = 0;
	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + U_acc * paraCarFolloingTimeStep_Modified_IMD);

	return UpdatedFollowingSpeed;
}