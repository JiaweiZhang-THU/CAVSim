#pragma once
#ifndef _LANE_CHANGING_
#define _LANE_CHANGING_

class LaneChanging
{
public:
	LaneChanging();
	~LaneChanging();

public:
	
	double LateralSpeed(double aDistannceFromTargetLane, int aChangeDirection);

};

#endif // !_LANE_CHANGING_
