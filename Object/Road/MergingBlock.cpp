#include "MergingBlock.h"
#include<list>
#include "MergingBlockParameters.h"

MergingBlock::MergingBlock()
{
	MergingBlock::Id = -1;

	MergingBlock::NumOfLanesOnMainRoad = paraNumOfLanesOnMainRoad;

	MergingBlock::LaneWidth = paraLaneWidthMerging;

	MergingBlock::EnterMainRoadLength = paraEnterMainRoadLength;

	MergingBlock::ExitMainRoadLength = paraExitMainRoadLength;

	MergingBlock::ConflictRoadLength = paraConflictRoadLength;

	MergingBlock::MergingLaneLength = paraMergingLaneLength;

	MergingBlock::AngleOfMergingLane = paraAngleOfMergingLane;

	MergingBlock::EnterMainRoadCenter = paraEnterMainRoadCenter;

	MergingBlock::SetStraightLaneInterface();

	MergingBlock::SetLaneUnitPointerNetwork();

	MergingBlock::SetEnteringVehicleInitialPositonAndBoseList();
}

MergingBlock::~MergingBlock()
{
}


void MergingBlock::SetStraightLaneInterface()
{
	StraightLaneBlock localNewBlock1(1, MergingBlock::NumOfLanesOnMainRoad, 0.0 * PI_MergingBlock, MergingBlock::EnterMainRoadLength);
	MergingBlock::Zone1_EnterMainRoad = localNewBlock1;
	BPointCoordinate aLocation_1(MergingBlock::EnterMainRoadCenter.X, MergingBlock::EnterMainRoadCenter.Y);
	MergingBlock::Zone1_EnterMainRoad.SetLocation(aLocation_1);

	StraightLaneBlock localNewBlock2(2, MergingBlock::NumOfLanesOnMainRoad, 0.0 * PI_MergingBlock, MergingBlock::ConflictRoadLength);
	MergingBlock::Zone2_ConflictMainRoad = localNewBlock2;
	BPointCoordinate aLocation_2(MergingBlock::EnterMainRoadCenter.X + MergingBlock::EnterMainRoadLength / 2 + MergingBlock::ConflictRoadLength / 2, MergingBlock::EnterMainRoadCenter.Y);
	MergingBlock::Zone2_ConflictMainRoad.SetLocation(aLocation_2);

	StraightLaneBlock localNewBlock3(3, MergingBlock::NumOfLanesOnMainRoad, 0.0 * PI_MergingBlock, MergingBlock::ExitMainRoadLength);
	MergingBlock::Zone3_ExitMainRoad = localNewBlock3;
	BPointCoordinate aLocation_3(MergingBlock::EnterMainRoadCenter.X + MergingBlock::EnterMainRoadLength / 2 + MergingBlock::ConflictRoadLength + MergingBlock::ExitMainRoadLength / 2, MergingBlock::EnterMainRoadCenter.Y);
	MergingBlock::Zone3_ExitMainRoad.SetLocation(aLocation_3);

	StraightLaneBlock localNewBlock4(4, 1, MergingBlock::AngleOfMergingLane, MergingBlock::MergingLaneLength);
	MergingBlock::Zone4_MergingLane = localNewBlock4;
	MergingBlock::Zone4_MergingLane.Direction = MergingBlock::AngleOfMergingLane;
	BPointCoordinate aLocation_4(MergingBlock::EnterMainRoadCenter.X + MergingBlock::EnterMainRoadLength / 2 + MergingBlock::LaneWidth / sin(MergingBlock::AngleOfMergingLane) - MergingBlock::MergingLaneLength * cos(MergingBlock::AngleOfMergingLane)/2, MergingBlock::EnterMainRoadCenter.Y - MergingBlock::MergingLaneLength * sin(MergingBlock::AngleOfMergingLane) / 2);
	MergingBlock::Zone4_MergingLane.SetLocation(aLocation_4);
}




void MergingBlock::SetLaneUnitPointerNetwork()
{
	LaneUnit* localLaneUnitPointer = NULL;
	list<LaneUnit>::iterator localLaneUnitIterator;
	list<StraightLaneBlock>::iterator localStraightLaneBlockIteratort;

	int localLaneUnitId = 1;
	for (localLaneUnitIterator = MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++, localLaneUnitId++)
	{
		localLaneUnitPointer = &(*localLaneUnitIterator);

		for (int i = 0; i < 2; i++)
		{
			for (int j = -1; j < 2; j++)
			{
				localLaneUnitPointer->AdjacentUnitPointer[3 * (i + 1) + (j + 1)] = MergingBlock::GetOneLaneUnitPointer(1 + i, localLaneUnitId + j);
				if (localLaneUnitId == 1 and i == 0 and j == -1)
				{
					localLaneUnitPointer->AdjacentUnitPointer[3 * (i + 1) + (j + 1)] = MergingBlock::GetOneLaneUnitPointer(4, 1);
				}
			}
		}
	}

	localLaneUnitId = 1;
	for (localLaneUnitIterator = MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++, localLaneUnitId++)
	{
		localLaneUnitPointer = &(*localLaneUnitIterator);

		for (int i = -1; i < 2; i++)
		{
			for (int j = -1; j < 2; j++)
			{
				localLaneUnitPointer->AdjacentUnitPointer[3 * (i + 1) + (j + 1)] = MergingBlock::GetOneLaneUnitPointer(2 + i, localLaneUnitId + j);
				if (localLaneUnitId == 1 and i == 0 and j == -1)
				{
					localLaneUnitPointer->AdjacentUnitPointer[3 * (i + 1) + (j + 1)] = MergingBlock::GetOneLaneUnitPointer(4, 1);
				}
			}
		}
	}

	localLaneUnitId = 1;
	for (localLaneUnitIterator = MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++, localLaneUnitId++)
	{
		localLaneUnitPointer = &(*localLaneUnitIterator);

		for (int i = -1; i < 1; i++)
		{
			for (int j = -1; j < 2; j++)
			{
				localLaneUnitPointer->AdjacentUnitPointer[3 * (i + 1) + (j + 1)] = MergingBlock::GetOneLaneUnitPointer(3 + i, localLaneUnitId + j);
			}
		}
	}

	localLaneUnitId = 1;
	for (localLaneUnitIterator = MergingBlock::Zone4_MergingLane.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone4_MergingLane.StraightMultiLanes.end(); localLaneUnitIterator++, localLaneUnitId++)
	{
		localLaneUnitPointer = &(*localLaneUnitIterator);
		localLaneUnitPointer->AdjacentUnitPointer[2] = MergingBlock::GetOneLaneUnitPointer(1, 1);
		localLaneUnitPointer->AdjacentUnitPointer[4] = MergingBlock::GetOneLaneUnitPointer(4, 1);
		localLaneUnitPointer->AdjacentUnitPointer[5] = MergingBlock::GetOneLaneUnitPointer(2, 1);
		localLaneUnitPointer->AdjacentUnitPointer[8] = MergingBlock::GetOneLaneUnitPointer(3, 1);
	}
}


LaneUnit* MergingBlock::GetOneLaneUnitPointer(int aZoneIndex, int aUnitIndex)
{
	list<LaneUnit>::iterator localLaneUnitIterator;
	if (aZoneIndex <= 0 || aZoneIndex > 4)
	{
		return NULL;
	}
	else {
		if (aZoneIndex == 1)
		{
			if (aUnitIndex <= 0 || aUnitIndex > MergingBlock::NumOfLanesOnMainRoad)
			{
				return NULL;
			}
			else
			{
				for (localLaneUnitIterator = MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++)
				{
					if (localLaneUnitIterator->Id == aUnitIndex)
					{
						return &(*localLaneUnitIterator);
					}
				}
			}
		}


		if (aZoneIndex == 2)
		{
			if (aUnitIndex <= 0 || aUnitIndex > MergingBlock::NumOfLanesOnMainRoad)
			{
				return NULL;
			}
			else
			{
				for (localLaneUnitIterator = MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++)
				{
					if (localLaneUnitIterator->Id == aUnitIndex)
					{
						return &(*localLaneUnitIterator);
					}
				}
			}
		}


		if (aZoneIndex == 3)
		{
			if (aUnitIndex <= 0 || aUnitIndex > MergingBlock::NumOfLanesOnMainRoad)
			{
				return NULL;
			}
			else
			{
				for (localLaneUnitIterator = MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++)
				{
					if (localLaneUnitIterator->Id == aUnitIndex)
					{
						return &(*localLaneUnitIterator);
					}
				}
			}
		}


		if (aZoneIndex == 4)
		{
			if (aUnitIndex != 1)
			{
				return NULL;
			}
			else
			{
				for (localLaneUnitIterator = MergingBlock::Zone4_MergingLane.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone4_MergingLane.StraightMultiLanes.end(); localLaneUnitIterator++)
				{
					if (localLaneUnitIterator->Id == aUnitIndex)
					{
						return &(*localLaneUnitIterator);
					}
				}
			}
		}
	}
	return NULL;
}


void MergingBlock::SetEnteringVehicleInitialPositonAndBoseList()
{
	list<LaneUnit>::iterator localItertorFistBlockMultiLanes = MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.begin();
	double localDelta = 0.05;
	for (int i = 0; i < MergingBlock::NumOfLanesOnMainRoad; i++)
	{	
		BNode aPositonAndPose(localItertorFistBlockMultiLanes->From_Node.Location.X + localDelta * cos(MergingBlock::Zone1_EnterMainRoad.Direction), localItertorFistBlockMultiLanes->From_Node.Location.Y + localDelta * sin(MergingBlock::Zone1_EnterMainRoad.Direction), MergingBlock::Zone1_EnterMainRoad.Direction);
		aPositonAndPose.Id = i + 1;
		MergingBlock::EnteringVehicleInitialPositonAndPoseList.push_back(aPositonAndPose);

		localItertorFistBlockMultiLanes++;
	}

	list<LaneUnit>::iterator localItertorFistBlockMultiLanes2 = MergingBlock::Zone4_MergingLane.StraightMultiLanes.begin();
	BNode aPositonAndPose2(localItertorFistBlockMultiLanes2->From_Node.Location.X + localDelta * cos(MergingBlock::Zone4_MergingLane.Direction), localItertorFistBlockMultiLanes2->From_Node.Location.Y + localDelta * sin(MergingBlock::Zone4_MergingLane.Direction), MergingBlock::Zone4_MergingLane.Direction);
	aPositonAndPose2.Id = MergingBlock::NumOfLanesOnMainRoad + 1;
	MergingBlock::EnteringVehicleInitialPositonAndPoseList.push_back(aPositonAndPose2);
}

void MergingBlock::SetIntersectionFourRectangle()
{
	double localBlock1X = 0.25 * (MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.front().From_Node.Location.X + MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.front().To_Node.Location.X + MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.back().From_Node.Location.X + MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.back().To_Node.Location.X);
	double localBlock1Y = 0.25 * (MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.front().From_Node.Location.Y + MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.front().To_Node.Location.Y + MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.back().From_Node.Location.Y + MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.back().To_Node.Location.Y);
	BPointCoordinate localOriginalBlock1Location(localBlock1X, localBlock1Y);
	BRectangle localRectangle1(
		MergingBlock::EnterMainRoadLength,
		MergingBlock::Zone1_EnterMainRoad.LanesNumber * MergingBlock::LaneWidth,
		localOriginalBlock1Location,
		MergingBlock::Zone1_EnterMainRoad.Direction
	);
	MergingBlock::IntersectionFourRectangle[0] = localRectangle1;

	localBlock1X = 0.25 * (MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.front().From_Node.Location.X + MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.front().To_Node.Location.X + MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.back().From_Node.Location.X + MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.back().To_Node.Location.X);
	localBlock1Y = 0.25 * (MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.front().From_Node.Location.Y + MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.front().To_Node.Location.Y + MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.back().From_Node.Location.Y + MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.back().To_Node.Location.Y);
	BPointCoordinate localOriginalBlock2Location(localBlock1X, localBlock1Y);
	BRectangle localRectangle2(
		MergingBlock::EnterMainRoadLength,
		MergingBlock::Zone2_ConflictMainRoad.LanesNumber * MergingBlock::LaneWidth,
		localOriginalBlock2Location,
		MergingBlock::Zone2_ConflictMainRoad.Direction
	);
	MergingBlock::IntersectionFourRectangle[1] = localRectangle2;

	localBlock1X = 0.25 * (MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.front().From_Node.Location.X + MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.front().To_Node.Location.X + MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.back().From_Node.Location.X + MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.back().To_Node.Location.X);
	localBlock1Y = 0.25 * (MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.front().From_Node.Location.Y + MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.front().To_Node.Location.Y + MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.back().From_Node.Location.Y + MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.back().To_Node.Location.Y);
	BPointCoordinate localOriginalBlock3Location(localBlock1X, localBlock1Y);
	BRectangle localRectangle3(
		MergingBlock::EnterMainRoadLength,
		MergingBlock::Zone3_ExitMainRoad.LanesNumber * MergingBlock::LaneWidth,
		localOriginalBlock3Location,
		MergingBlock::Zone3_ExitMainRoad.Direction
	);
	MergingBlock::IntersectionFourRectangle[2] = localRectangle3;

	localBlock1X = 0.25 * (MergingBlock::Zone4_MergingLane.StraightMultiLanes.front().From_Node.Location.X + MergingBlock::Zone4_MergingLane.StraightMultiLanes.front().To_Node.Location.X + MergingBlock::Zone4_MergingLane.StraightMultiLanes.back().From_Node.Location.X + MergingBlock::Zone4_MergingLane.StraightMultiLanes.back().To_Node.Location.X);
	localBlock1Y = 0.25 * (MergingBlock::Zone4_MergingLane.StraightMultiLanes.front().From_Node.Location.Y + MergingBlock::Zone4_MergingLane.StraightMultiLanes.front().To_Node.Location.Y + MergingBlock::Zone4_MergingLane.StraightMultiLanes.back().From_Node.Location.Y + MergingBlock::Zone4_MergingLane.StraightMultiLanes.back().To_Node.Location.Y);
	BPointCoordinate localOriginalBlock4Location(localBlock1X, localBlock1Y);
	BRectangle localRectangle4(
		MergingBlock::EnterMainRoadLength,
		MergingBlock::Zone4_MergingLane.LanesNumber * MergingBlock::LaneWidth,
		localOriginalBlock4Location,
		MergingBlock::Zone4_MergingLane.Direction
	);
	MergingBlock::IntersectionFourRectangle[3] = localRectangle4;
	
}

bool MergingBlock::IsInIntersection(double aLocationX, double aLocationY)
{
	for (int i = 0; i < 4; i++)
	{
		if (MergingBlock::IntersectionFourRectangle[i].IsPointInRect(aLocationX, aLocationY) == true)
		{
			return true;
		}
	}
	return false;
}


LaneUnit* MergingBlock::FindLaneUnitPointer(int BlockId, int LaneUnitId)
{
	list<LaneUnit>::iterator localLaneUnitIterator;

	int j = 0;
	if (BlockId == 1)
	{
		for (localLaneUnitIterator = MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone1_EnterMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++)
		{
			j += 1;
			if (j == LaneUnitId)
			{
				break;
			}
		}
	}
	

	j = 0;
	if (BlockId == 2)
	{
		for (localLaneUnitIterator = MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone2_ConflictMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++)
		{
			j += 1;
			if (j == LaneUnitId)
			{
				break;
			}
		}
	}

	j = 0;
	if (BlockId == 3)
	{
		for (localLaneUnitIterator = MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone3_ExitMainRoad.StraightMultiLanes.end(); localLaneUnitIterator++)
		{
			j += 1;
			if (j == LaneUnitId)
			{
				break;
			}
		}
	}


	j = 0;
	if (BlockId == 4)
	{
		for (localLaneUnitIterator = MergingBlock::Zone4_MergingLane.StraightMultiLanes.begin(); localLaneUnitIterator != MergingBlock::Zone4_MergingLane.StraightMultiLanes.end(); localLaneUnitIterator++)
		{
			j += 1;
			if (j == LaneUnitId)
			{
				break;
			}
		}
	}

	return &(*localLaneUnitIterator);
}