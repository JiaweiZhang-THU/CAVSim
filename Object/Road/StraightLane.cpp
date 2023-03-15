#include "StraightLane.h"

#include "StraightLaneParameters.h"

StraightLane::StraightLane()
{
	StraightLane::StraightLaneBlockNum = paraStraightLaneBlockNum;
	StraightLane::LanesNumber = paraDefaultLaneNumBasedOnBlocakParameter;

	StraightLane::FirstBlockCenter = paraFirstBlockCenter;
	StraightLane::Direction = paraStraightLaneDirection;

	StraightLane::SetStraightLaneList();
	StraightLane::SetLaneUnitPointerNetwork();

	StraightLane::LanesNumber = StraightLane::StraightLaneList.begin()->LanesNumber;

	StraightLane::SetEnteringVehicleInitialPositonAndBoseList();

	StraightLane::TrafficFlowRateArray.fill(0);

	StraightLane::CurrentMovingVehiclesNum = 0;
	StraightLane::InitialStatisticalVariable();
}


StraightLane::StraightLane(int aStraightLaneBlockNum)
{
	StraightLane::StraightLaneBlockNum = aStraightLaneBlockNum;
	StraightLane::LanesNumber = paraDefaultLaneNumBasedOnBlocakParameter;

	StraightLane::FirstBlockCenter = paraFirstBlockCenter;
	StraightLane::Direction = paraStraightLaneDirection;

	StraightLane::SetStraightLaneList();
	StraightLane::SetLaneUnitPointerNetwork();

	StraightLane::LanesNumber = StraightLane::StraightLaneList.begin()->LanesNumber;

	StraightLane::SetEnteringVehicleInitialPositonAndBoseList();

	StraightLane::TrafficFlowRateArray.fill(0);

	StraightLane::CurrentMovingVehiclesNum = 0;
	StraightLane::InitialStatisticalVariable();
}

StraightLane::StraightLane(int aStraightLaneBlockNum, double aDirection)
{
	StraightLane::StraightLaneBlockNum = aStraightLaneBlockNum;
	StraightLane::LanesNumber = paraDefaultLaneNumBasedOnBlocakParameter;

	StraightLane::Direction = aDirection;
	StraightLane::FirstBlockCenter = paraFirstBlockCenter;

	StraightLane::SetStraightLaneList();
	StraightLane::SetLaneUnitPointerNetwork();

	StraightLane::LanesNumber = StraightLane::StraightLaneList.begin()->LanesNumber;

	StraightLane::SetEnteringVehicleInitialPositonAndBoseList();

	StraightLane::TrafficFlowRateArray.fill(0);

	StraightLane::CurrentMovingVehiclesNum = 0;
	StraightLane::InitialStatisticalVariable();
}

StraightLane::StraightLane(int aStraightLaneBlockNum, double aDirection,  BPointCoordinate aFirstBlockCenter)
{
	StraightLane::StraightLaneBlockNum = aStraightLaneBlockNum;
	StraightLane::LanesNumber = paraDefaultLaneNumBasedOnBlocakParameter;

	StraightLane::FirstBlockCenter = aFirstBlockCenter;
	StraightLane::Direction = aDirection;

	StraightLane::SetStraightLaneList();
	StraightLane::SetLaneUnitPointerNetwork();

	StraightLane::LanesNumber = StraightLane::StraightLaneList.begin()->LanesNumber;

	StraightLane::SetEnteringVehicleInitialPositonAndBoseList();

	StraightLane::TrafficFlowRateArray.fill(0);

	StraightLane::CurrentMovingVehiclesNum = 0;
	StraightLane::InitialStatisticalVariable();
}

void StraightLane::SetStraightLaneList()
{
	for (int i = 0; i < StraightLane::StraightLaneBlockNum; i++)
	{
		StraightLaneBlock aStraightLaneBlock(i+1, StraightLane::LanesNumber, StraightLane::Direction, paraStraightPerBlockLength);
		StraightLane::StraightLaneList.push_back(aStraightLaneBlock);
	}

	list<StraightLaneBlock>::iterator aIterIndex;
	double i = 0;
	for (aIterIndex = StraightLane::StraightLaneList.begin(); aIterIndex != StraightLane::StraightLaneList.end(); aIterIndex++)
	{
		i += 1;

		aIterIndex->Direction = StraightLane::Direction;

		BPointCoordinate aLocation(StraightLane::FirstBlockCenter.X +( (i - 1) * paraStraightPerBlockLength) * cos(StraightLane::Direction), StraightLane::FirstBlockCenter.Y + ((i - 1) * paraStraightPerBlockLength) * sin(StraightLane::Direction));
		aIterIndex->SetLocation(aLocation);
	}

	StraightLane::Location.X = 0.25 * (StraightLane::StraightLaneList.front().StraightMultiLanes.front().From_Node.Location.X + StraightLane::StraightLaneList.front().StraightMultiLanes.back().From_Node.Location.X + StraightLane::StraightLaneList.back().StraightMultiLanes.front().To_Node.Location.X + StraightLane::StraightLaneList.back().StraightMultiLanes.back().To_Node.Location.X);
	StraightLane::Location.Y = 0.25 * (StraightLane::StraightLaneList.front().StraightMultiLanes.front().From_Node.Location.Y + StraightLane::StraightLaneList.front().StraightMultiLanes.back().From_Node.Location.Y + StraightLane::StraightLaneList.back().StraightMultiLanes.front().To_Node.Location.Y + StraightLane::StraightLaneList.back().StraightMultiLanes.back().To_Node.Location.Y);

	StraightLane::Width = StraightLane::LanesNumber * paraPerLaneWidth;
	StraightLane::Length = StraightLane::StraightLaneBlockNum * paraStraightPerBlockLength;

	BRectangle localRoadRectangle(StraightLane::Width, StraightLane::Length, StraightLane::Location, StraightLane::Direction);
	StraightLane::StraightRectangle = localRoadRectangle;
}

void StraightLane::SetLaneUnitPointerNetwork()
{
	for (int i = 1; i <= StraightLane::StraightLaneBlockNum; i++)
	{
		for (int j = 1; j <= StraightLane::LanesNumber; j++)
		{
			StraightLane::SetOneLaneUnitPointerNetwork(i, j);
		}
	}
}

void StraightLane::SetOneLaneUnitPointerNetwork(int aBlockIndex, int aUnitIndex)
{
	LaneUnit* localLaneUnitPointer=NULL;
	list<LaneUnit>::iterator localLaneUnitIterator;
	list<StraightLaneBlock>::iterator localStraightLaneBlockIteratort;

	for (localStraightLaneBlockIteratort = StraightLane::StraightLaneList.begin(); localStraightLaneBlockIteratort != StraightLane::StraightLaneList.end(); localStraightLaneBlockIteratort++)
	{
		if (localStraightLaneBlockIteratort->Id == aBlockIndex)
		{
			for (localLaneUnitIterator = localStraightLaneBlockIteratort->StraightMultiLanes.begin(); localLaneUnitIterator != localStraightLaneBlockIteratort->StraightMultiLanes.end(); localLaneUnitIterator++)
			{
				if (localLaneUnitIterator->Id == aUnitIndex)
				{
					localLaneUnitPointer = &(*localLaneUnitIterator);
				}
			}
		}
	}
	for (int i = -1; i < 2; i++)
	{
		for (int j = -1; j < 2; j++)
		{
			localLaneUnitPointer->AdjacentUnitPointer[3 * (i + 1) + (j + 1)] = StraightLane::GetOneLaneUnitPointer(aBlockIndex + i, aUnitIndex + j);
		}
	}
}

LaneUnit* StraightLane::GetOneLaneUnitPointer(int aBlockIndex, int aUnitIndex)
{
	list<LaneUnit>::iterator localLaneUnitIterator;
	list<StraightLaneBlock>::iterator localStraightLaneBlockIteratort;
	if (aBlockIndex <= 0 || aBlockIndex > StraightLane::StraightLaneBlockNum)
	{
		return NULL;
	}
	else {
		if (aUnitIndex <= 0 || aUnitIndex > StraightLane::LanesNumber)
		{
			return NULL;
		}

		else {
			for (localStraightLaneBlockIteratort = StraightLane::StraightLaneList.begin(); localStraightLaneBlockIteratort != StraightLane::StraightLaneList.end(); localStraightLaneBlockIteratort++)
			{
				if (localStraightLaneBlockIteratort->Id == aBlockIndex)
				{
					for (localLaneUnitIterator = localStraightLaneBlockIteratort->StraightMultiLanes.begin(); localLaneUnitIterator != localStraightLaneBlockIteratort->StraightMultiLanes.end(); localLaneUnitIterator++)
					{
						if (localLaneUnitIterator->Id == aUnitIndex)
						{
							return &(*localLaneUnitIterator);
						}
					}
				}
			}
		}
	}
	return NULL;
}

string StraightLane::GetName()
{
	string Name = to_string(StraightLane::Id)+" Straight Lane :\n";
	
	list<StraightLaneBlock>::iterator aIterIndex;
	double i = 0;
	for (aIterIndex = StraightLane::StraightLaneList.begin(); aIterIndex != StraightLane::StraightLaneList.end(); aIterIndex++)
	{
		
		Name = Name + aIterIndex->GetName();
	}

	return Name;
}

void StraightLane::SetEnteringVehicleInitialPositonAndBoseList()
{
	list<LaneUnit>::iterator localItertorFistBlockMultiLanes = StraightLane::StraightLaneList.begin()->StraightMultiLanes.begin();

	for (int i = 0; i < StraightLane::LanesNumber; i++)
	{
		double localDelta = 0.05;
		BNode aPositonAndPose(localItertorFistBlockMultiLanes->From_Node.Location.X + localDelta * cos(StraightLane::Direction), localItertorFistBlockMultiLanes->From_Node.Location.Y + localDelta * sin(StraightLane::Direction), StraightLane::Direction);
		aPositonAndPose.Id = i+1;
		aPositonAndPose.RoadBlockType = 1;
		StraightLane::EnteringVehicleInitialPositonAndPoseList.push_back(aPositonAndPose);

		localItertorFistBlockMultiLanes++;
	}
}

LaneUnit* StraightLane::FindLaneUnitPointer(int BlockId, int LaneUnitId)
{
	list<StraightLaneBlock>::iterator localBlockIterator;
	list<LaneUnit>::iterator localLaneUnitIterator;

	int i = 0;
	for (localBlockIterator = StraightLane::StraightLaneList.begin(); localBlockIterator != StraightLane::StraightLaneList.end(); localBlockIterator++)
	{
		i += 1;
		if (i == BlockId)
		{
			break;
		}
	}

	int j = 0;
	for (localLaneUnitIterator = localBlockIterator->StraightMultiLanes.begin(); localLaneUnitIterator != localBlockIterator->StraightMultiLanes.end(); localLaneUnitIterator++)
	{
		j += 1;
		if (j == LaneUnitId)
		{
			break;
		}
	}

	return &(*localLaneUnitIterator);
}

void StraightLane::UpdateTrafficFlowRateArray(double aTimeInterval)
{
	list<StraightLaneBlock>::iterator localStraightLaneBlockIterator;
	int i = 0;
	for (localStraightLaneBlockIterator = StraightLane::StraightLaneList.begin(); localStraightLaneBlockIterator != StraightLane::StraightLaneList.end(); localStraightLaneBlockIterator++)
	{
		StraightLane::TrafficFlowRateArray[i] = localStraightLaneBlockIterator->GetTrafficFlowRate(aTimeInterval);
		i += 1;
	}
}

void StraightLane::InitialStatisticalVariable()
{
	StraightLane::EnteringVehiclesNum = 0;
	StraightLane::ExitingVehiclesNum = 0;
}