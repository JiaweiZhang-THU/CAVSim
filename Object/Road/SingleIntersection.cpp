#include "SingleIntersection.h"
#include "SingleIntersectionParameters.h"

SingleIntersection::SingleIntersection()
{
	SingleIntersection::SetIntersection();
}

void SingleIntersection::SetIntersection()
{
	SingleIntersection::StraightBlockInterfaceLength = paraStraightBlockInterfaceLength;
	Intersection* localIntersection0 = new Intersection(paraIntersection0LaneNumber, paraIntersection0Center, paraIntersection0Direction, paraStraightBlockInterfaceLength);
	SingleIntersection::Intersection0 = *localIntersection0;
	delete localIntersection0;
	SingleIntersection::Intersection0.Id = 0;
}

list<BNode> SingleIntersection::GetEnteringVehicleInitialPositonAndBoseList()
{

	list<BNode> localListBNodeIntersection0 = SingleIntersection::Intersection0.EnteringVehicleInitialPositonAndPoseList;
	return localListBNodeIntersection0;
}

int SingleIntersection::WhichRoadBlockKind(BPointCoordinate aLocation)
{
	int localRoadBlockType = 2;
	return localRoadBlockType;
}

bool SingleIntersection::IsInGriddedMap(BPointCoordinate aLocation)
{
	bool localRoadBlockType = false;

	if (SingleIntersection::Intersection0.IsInIntersection(aLocation) == true)
	{
		localRoadBlockType = true;
	}
	else
	{
		localRoadBlockType = false;
	}
	return localRoadBlockType;
}

BNode SingleIntersection::OriginalRoadNodePassingIntersection(Vehicle aVehicle)
{
	if (SingleIntersection::Intersection0.IsInIntersection(aVehicle.Location) == true)
	{
		BNode TargetNode = SingleIntersection::Intersection0.DrivingOutPlace(aVehicle.BlockId, aVehicle.LaneId);
		return TargetNode;
	}
}

int SingleIntersection::AssignSteeringType(int aVehicleId, int aLaneId)
{
	int localSteeringType = 0;
	double localRandomValue = MyRandom.randomUniform(0, 1);

	if (aLaneId == 1)
	{
		if (localRandomValue < 0.5)
		{
			localSteeringType = 1;
		}
		else
		{
			localSteeringType = 0;
		}
	}
	else
	{
		if (aLaneId == paraIntersection0LaneNumber)
		{

			if (localRandomValue < 0.5)
			{
				localSteeringType = -1;
			}
			else
			{
				localSteeringType = 0;
			}
		}
		else
		{
			localSteeringType = 0;
		}
	}
	return localSteeringType;
}

BNode SingleIntersection::NextRoadNodePassingIntersection(Vehicle& aVehicle)
{
	double localRandomValue = MyRandom.randomUniform(0, 1);
	int localSteeringType = 0; 
	localSteeringType = SingleIntersection::AssignSteeringType(aVehicle.Id, aVehicle.LaneId);


	aVehicle.SteeringType = localSteeringType;

	if (SingleIntersection::Intersection0.IsInIntersection(aVehicle.Location) == true)
	{
		if (localSteeringType != 0)
		{
			aVehicle.SteeringRadius = SingleIntersection::Intersection0.GetSteerRadius(aVehicle.LaneId, localSteeringType);
			aVehicle.SteeringCenter = SingleIntersection::Intersection0.GetSteerCenter(aVehicle.BlockId, localSteeringType);

			aVehicle.LeftInterDistance = 0.25 * 2 * acos(-1) * aVehicle.SteeringRadius;
		}
		else
		{
			aVehicle.LeftInterDistance = SingleIntersection::Intersection0.NumOfOneWayLanes * SingleIntersection::Intersection0.LaneWidth;
		}
		int localTargetBlockId = SingleIntersection::Intersection0.SteerTargetBlock(aVehicle, localSteeringType);
		BNode TargetNode = SingleIntersection::Intersection0.DrivingInPlace(localTargetBlockId, aVehicle.LaneId);

		aVehicle.EnteringBlockId = aVehicle.BlockId;
		aVehicle.TargetBlockId = localTargetBlockId;
		aVehicle.SteeringDriectionType = SingleIntersection::Intersection0.WhichTurnDirectionType(aVehicle.EnteringBlockId, aVehicle.TargetBlockId);

		aVehicle.FirstLaneId = (aVehicle.BlockId - 1) * SingleIntersection::Intersection0.NumOfOneWayLanes + aVehicle.LaneId;
		aVehicle.SecondLaneId = (localTargetBlockId - 1) * SingleIntersection::Intersection0.NumOfOneWayLanes + aVehicle.LaneId;

		return TargetNode;
	}
}