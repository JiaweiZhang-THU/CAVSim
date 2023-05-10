#ifndef _PASSING_INTERSECTION_
#define _PASSING_INTERSECTION_
#include "../../Object/Vehicle/Vehicle.h"

class PassingIntersection
{
public:
	PassingIntersection();
	~PassingIntersection();

	double XSpeed(double aTime, double aDeltaX);

	double YSpeed(double aTime, double aDeltaY);

	double GetAngularSpeedForSteeringLeft(double aRaduis, double aSteeringSpeed,double aTimeStep, double& aPoseAngle);

	double GetAngularSpeedForSteeringRight(double aRaduis, double aSteeringSpeed, double aTimeStep, double& aPoseAngle);

	void SpeedPlanning(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle);

	BPointCoordinate RotationCoordiante(BPointCoordinate aOriginalCoordinate, double aTheta);

};
#endif // !_PASSING_INTERSECTION_
