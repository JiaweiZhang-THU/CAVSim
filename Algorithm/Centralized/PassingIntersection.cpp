#include "PassingIntersection.h"
#include<iostream>

double PI = acos(-1);

PassingIntersection::PassingIntersection()
{
}

PassingIntersection::~PassingIntersection()
{
}

BPointCoordinate PassingIntersection::RotationCoordiante(BPointCoordinate aOriginalCoordinates, double aTheta)
{
	BPointCoordinate NewCoordinate;
	NewCoordinate.X = aOriginalCoordinates.X * cos(aTheta) - aOriginalCoordinates.Y * sin(aTheta);
	NewCoordinate.Y = aOriginalCoordinates.X * sin(aTheta) + aOriginalCoordinates.Y * cos(aTheta);
	return NewCoordinate;
}

double PassingIntersection::XSpeed(double aTime, double aDeltaX)
{
	return aDeltaX / (aTime + 0.0000001);
}

double PassingIntersection::YSpeed(double aTime, double aDeltaY)
{
	return aDeltaY / (aTime + 0.0000001);
}

double PassingIntersection::GetAngularSpeedForSteeringLeft(double aRaduis, double aSteeringSpeed,double aTimeStep, double& aPoseAngle)
{
	double localAngularSpeed;
	double localTheta = 2 * asin(aSteeringSpeed * aTimeStep / 2 / aRaduis);
	if (aPoseAngle + localTheta > PI / 2)
	{
		localAngularSpeed = (PI / 2 - aPoseAngle) / aTimeStep;
	}
	else
	{
		localAngularSpeed = localTheta / aTimeStep;
	}
	aPoseAngle += localAngularSpeed * aTimeStep;

	return localAngularSpeed;
}

double PassingIntersection::GetAngularSpeedForSteeringRight(double aRaduis, double aSteeringSpeed, double aTimeStep, double& aPoseAngle)
{
	double localAngularSpeed;
	double localTheta = 2 * asin(aSteeringSpeed * aTimeStep / 2 / aRaduis);
	if (aPoseAngle - localTheta < -PI / 2)
	{
		localAngularSpeed = (-PI / 2 - aPoseAngle) / aTimeStep;
	}
	else
	{
		localAngularSpeed = -localTheta / aTimeStep;
	}
	aPoseAngle += localAngularSpeed * aTimeStep;

	return localAngularSpeed;
}

void PassingIntersection::SpeedPlanning(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle)
{
	BPointCoordinate localTargetNodeLocation(aVehicle.TargetNode.Location.X - aVehicle.Location.X, aVehicle.TargetNode.Location.Y - aVehicle.Location.Y);
	localTargetNodeLocation= PassingIntersection::RotationCoordiante(localTargetNodeLocation, -aVehicle.Direction);

	double localDeltaX = localTargetNodeLocation.X;
	double localDeltaY = localTargetNodeLocation.Y;

	double localLaturalSpeed;
	double localLongitudeSpeed;
	
	localLaturalSpeed = PassingIntersection::XSpeed(aVehicle.TimeCostToTargetNede, localDeltaY);
	localLongitudeSpeed = PassingIntersection::YSpeed(aVehicle.TimeCostToTargetNede, localDeltaX);

	if (aVehicle.SteeringType == -1)
	{
		PassingIntersection::GetAngularSpeedForSteeringLeft(aVehicle.SteeringRadius, aVehicle.MaxSteerSpeed, aVehicle.TimeStep, AgentVehiclePoseAngle);
	}
	else
	{
		if (aVehicle.SteeringType == 1)
		{
			PassingIntersection::GetAngularSpeedForSteeringRight(aVehicle.SteeringRadius, aVehicle.MaxSteerSpeed, aVehicle.TimeStep, AgentVehiclePoseAngle);
		}
		else
		{
			AgentVehiclePoseAngle = 0;
		}
	}
	AgentVehicleSpeed = aVehicle.MaxSteerSpeed;
}


