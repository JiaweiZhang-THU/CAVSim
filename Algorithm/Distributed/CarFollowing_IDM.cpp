#include "CarFollowing_IDM.h"
#include "CarFollowingIDMParameters.h"

CarFollowingIDM::CarFollowingIDM()
{

	CarFollowingIDM::CarFolloingMantainDistance = IDM_PARAMETERS_So;
}

CarFollowingIDM::CarFollowingIDM(double aCarFolloingMaintainingDistance)
{
	CarFollowingIDM::CarFolloingMantainDistance = aCarFolloingMaintainingDistance;
}


CarFollowingIDM::~CarFollowingIDM()
{
}

double CarFollowingIDM::FreeDriving(double aFollowingSpeed, Vehicle AgentVehicle)
{
	double UpdatedFollowingSpeed = 0;

	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + AgentVehicle.MaxStraightAccel * paraCarFolloingTimeStep_IMD);

	return UpdatedFollowingSpeed;

}

double CarFollowingIDM::FreeDrivingForStringStability(double aFollowingSpeed, Vehicle AgentVehicle, double aTimeStep)
{
	double UpdatedFollowingSpeed = 0;

	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + AgentVehicle.MaxStraightAccel * aTimeStep);

	return UpdatedFollowingSpeed;

}

double CarFollowingIDM::AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle)
{

	double local_IDM_PARAMETERS_a = AgentVehicle.MaxStraightAccel;
	double S_star = IDM_PARAMETERS_So + fmax(0, aFollowingSpeed * IDM_PARAMETERS_To + aFollowingSpeed * (aFollowingSpeed - aHeadSpeed) / (2 * sqrt(local_IDM_PARAMETERS_a * IDM_PARAMETERS_b)));

	double U_acc = local_IDM_PARAMETERS_a * (1 - pow(aFollowingSpeed / AgentVehicle.MaxStraightSpeed, IDM_PARAMETERS_Delta) - pow(S_star/aHeadDistance,2));

	double UpdatedFollowingSpeed = 0;
	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + U_acc * paraCarFolloingTimeStep_IMD);
	UpdatedFollowingSpeed = fmax(UpdatedFollowingSpeed, 0);

	return UpdatedFollowingSpeed;

}


double CarFollowingIDM::AgentBasedCarFollowing(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, Vehicle AgentVehicle, double& aHeadwayDistance)
{
	double local_IDM_PARAMETERS_a = AgentVehicle.MaxStraightAccel;
	double S_star = IDM_PARAMETERS_So + fmax(0, aFollowingSpeed * IDM_PARAMETERS_To + aFollowingSpeed * (aFollowingSpeed - aHeadSpeed) / (2 * sqrt(local_IDM_PARAMETERS_a * IDM_PARAMETERS_b)));
	aHeadwayDistance = S_star;

	double U_acc = local_IDM_PARAMETERS_a * (1 - pow(aFollowingSpeed / AgentVehicle.MaxStraightSpeed, IDM_PARAMETERS_Delta) - pow(S_star / aHeadDistance, 2));


	double UpdatedFollowingSpeed = 0;
	UpdatedFollowingSpeed = fmin(AgentVehicle.MaxStraightSpeed, aFollowingSpeed + U_acc * paraCarFolloingTimeStep_IMD);

	return UpdatedFollowingSpeed;
}

double CarFollowingIDM::AgentBasedCarFollowingForStringStability(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, double aAccelerationLastStep, Vehicle AgentVehicle, double aTimeStep, double& aHeadwayDistance, double& aAcceleration)
{
	double local_MaxAcceleration = AgentVehicle.MaxStraightAccel;
	double S_star = IDM_PARAMETERS_So + fmax(0, aFollowingSpeed * IDM_PARAMETERS_To + aFollowingSpeed * (aFollowingSpeed - aHeadSpeed) / (2 * sqrt(local_MaxAcceleration * IDM_PARAMETERS_b)));
	aHeadwayDistance = S_star; 

	double U_acc = local_MaxAcceleration * (1 - pow(aFollowingSpeed / AgentVehicle.MaxStraightSpeed, IDM_PARAMETERS_Delta) - pow(S_star / aHeadDistance, 2));
	aAcceleration = U_acc;

	double UpdatedFollowingSpeed = 0;
	UpdatedFollowingSpeed = aFollowingSpeed + 0.5 * (aAccelerationLastStep + aAcceleration) * aTimeStep;

	return UpdatedFollowingSpeed;
}


double CarFollowingIDM::AgentBasedCarFollowingForStringStability(double aHeadDistance, double aHeadSpeed, double aFollowingSpeed, double aAccelerationLastStep, Vehicle AgentVehicle, double aTimeStep, double& aHeadwayDistance, double& aAcceleration, double aTimeHeadway_TH)
{
	double local_MaxAcceleration = AgentVehicle.MaxStraightAccel;
	double S_star = IDM_PARAMETERS_So + fmax(0, aFollowingSpeed * aTimeHeadway_TH + aFollowingSpeed * (aFollowingSpeed - aHeadSpeed) / (2 * sqrt(local_MaxAcceleration * IDM_PARAMETERS_b)));
	aHeadwayDistance = S_star;

	double U_acc = local_MaxAcceleration * (1 - pow(aFollowingSpeed / AgentVehicle.MaxStraightSpeed, IDM_PARAMETERS_Delta) - pow(S_star / aHeadDistance, 2));
	aAcceleration = U_acc;

	double UpdatedFollowingSpeed = 0;
	UpdatedFollowingSpeed = aFollowingSpeed + 0.5 * (aAccelerationLastStep + aAcceleration) * aTimeStep;

	return UpdatedFollowingSpeed;
}
