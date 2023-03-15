#pragma once

double paraTimeStepForStringStability = 0.01;

double paraTimeIntervalForUpdateTrafficFlowRateForStringStability = 400; 

double paraVehileFlowPossionLambdaPerLaneForStringStability = 2000.0 / (3600 * 5 * 10);

int paraPerFlowVehicleNumForStringStability = 1000;

int paraDefaultLaneNumDefinedSLBPForStringStability = 2;

double paraLaneWidthInSimulatorForStringStability = 4;

double paraMaxExpectStraightSpeedForStringStability = 110 / 3.6; 

double paraMinExpectStraightSpeedForStringStability = 40 / 3.6;

double paraGammaDistribution_alphaForStringStability = 18.75;

double paraGammaDistribution_lambedForStringStability = 4;

double paraLengthLaneChangingNotPermittedForStringStability = 50;

