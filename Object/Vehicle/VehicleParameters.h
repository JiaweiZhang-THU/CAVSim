#pragma once
#ifndef _VEHICLE_PARAMETERS_
#define _VEHICLE_PARAMETERS_
#include"..\Road\BPointCoordinate.h"

double paraVehicleLength = 4;
double paraVehicleWidth = 2;
int paraVehicleType = -1;
double paraMaxStraightSpeed = 8;
double paraMinStraightSpeed = 0;

double paraMaxStraightAccel = 1;
double paraMinStraightAccel = -1;

double paraMaxSteerSpeed = 8;
double paraMaxAngularAccel = 10;

double paraMaxAngular = 5;

double paraDefaultSpeed = 0;
double paraDefaultAngularSpeed = 0;
double paraDefaultPoseAngle = 0;
double paraDefaultRoadDirection = 0;
BPointCoordinate paraDefaultLocation(0, 0);
double paraLaneUnitWidthDefinedInLaneUnit = 4; 
double paraDistanceNoise = 0.05;

int paraDefaultLaneNumDefinedInStraightBlock = 2;
double paraHLCMPerceptionDistance = 20; 
double paraHLCMPerceptionHeadDistance = 15;
double paraHLCMPerceptionTailDistance = 5;
double paraHLCMPerceptionDistanceNoise = 0.001;


int paraHLCMWidthOfPerceptionMatrix = 3;
int paraHLCMLengthOfPerceptionMatrix = 8;


int paraPerceptualDelayStepsNumInVehicle = 5;

double IDM_Parameters_b_E = 2;

double IDM_Parameters_b_sigma = 0.3;

double IDM_Parameters_T0_E = 1.5;

double IDM_Parameters_T0_sigma = 0.3;

#endif // !_VEHICLE_PARAMETERS_
