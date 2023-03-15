#ifndef _STRAIGHT_LANE_
#define _STRAIGHT_LANE_

#pragma once
#include<list>
#include<array>

#include "StraightLaneBlock.h"
#include "BPointCoordinate.h"
#include "BItem.h"
class StraightLane: public BItem
{
public:
	int StraightLaneBlockNum;

	int LanesNumber;

	BPointCoordinate Location;
	double Width;
	double Length;

	BRectangle StraightRectangle;

	BPointCoordinate FirstBlockCenter;
	
	double Direction;

	list<StraightLaneBlock> StraightLaneList;

	array<double, 1000> TrafficFlowRateArray;
	void UpdateTrafficFlowRateArray(double aTimeInterval);

	void InitialStatisticalVariable();

	int EnteringVehiclesNum;

	int ExitingVehiclesNum;

	int CurrentMovingVehiclesNum;

	StraightLane();
	StraightLane(int aStraightLaneBlockNum);
	StraightLane(int aStraightLaneBlockNum, double aDirection);
	StraightLane(int aStraightLaneBlockNum, double aDirection, BPointCoordinate aFirstBlockCenter);

	void SetStraightLaneList();

	void SetLaneUnitPointerNetwork();
	void SetOneLaneUnitPointerNetwork(int aBlockIndex, int aUnitIndex);
	LaneUnit* GetOneLaneUnitPointer(int aBlockIndex, int aUnitIndex);

	string GetName();

	list<BNode> EnteringVehicleInitialPositonAndPoseList;
	void SetEnteringVehicleInitialPositonAndBoseList();

	LaneUnit* FindLaneUnitPointer(int BlockId, int LaneUnitId);

};

#endif // !_STRAIGHT_LANE_



