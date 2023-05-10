#include "LaneChanging.h"
#include "LaneChangingParameters.h"

LaneChanging::LaneChanging()
{
}

LaneChanging::~LaneChanging()
{
}

double LaneChanging::LateralSpeed(double aDistannceFromTargetLane, int aChangeDirection)
{
	double localLateralSpeed;
	double localConstantSpeedForLaneChanging = paraLaneWidthInLaneChanging / paraTimeChanging;
	if (aChangeDirection == 0 || aDistannceFromTargetLane == 0)
	{
		localLateralSpeed = 0;
		return localLateralSpeed;
	}
	else
	{
		if (aChangeDirection == 1)
		{
			if (aDistannceFromTargetLane > 0)
			{
				localLateralSpeed = localConstantSpeedForLaneChanging;
			}
			else
			{
				localLateralSpeed = -1 * localConstantSpeedForLaneChanging;
			}
			return localLateralSpeed;
		}
		else
		{
			if (aDistannceFromTargetLane > 0)
			{
				localLateralSpeed = -1 * localConstantSpeedForLaneChanging;
			}
			else
			{
				localLateralSpeed = localConstantSpeedForLaneChanging;
			}
			return localLateralSpeed;
		}
	}
}