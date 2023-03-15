#ifndef _MERGINGBLOCK_
#define _MERGINGBLOCK_

#include<set>
#include "StraightLaneBlock.h"

const double PI_MergingBlock = acos(-1);

class MergingBlock: public BItem
{
public:
	MergingBlock();
	~MergingBlock();

	int NumOfLanesOnMainRoad;

	double LaneWidth;

	double EnterMainRoadLength;
	double ExitMainRoadLength;
	double ConflictRoadLength;
	double MergingLaneLength;

	BPointCoordinate EnterMainRoadCenter;
	BPointCoordinate ConflictBoundaryPointOnMainRoad;
	BPointCoordinate ConflictBoundaryPointOnRamp;

	double AngleOfMergingLane;

	StraightLaneBlock Zone1_EnterMainRoad;
	StraightLaneBlock Zone2_ConflictMainRoad;
	StraightLaneBlock Zone3_ExitMainRoad;
	StraightLaneBlock Zone4_MergingLane;
	void SetStraightLaneInterface();

	void SetLaneUnitPointerNetwork();
	LaneUnit* GetOneLaneUnitPointer(int aZoneIndex, int aUnitIndex);

	BRectangle IntersectionFourRectangle[4];
	void SetIntersectionFourRectangle();

	bool IsInIntersection(double aLocationX, double aLocationY);

	list<BNode> EnteringVehicleInitialPositonAndPoseList;
	void SetEnteringVehicleInitialPositonAndBoseList();

	LaneUnit* FindLaneUnitPointer(int BlockId, int LaneUnitId);

private:

};
#endif // !_MERGINGBLOCK_
