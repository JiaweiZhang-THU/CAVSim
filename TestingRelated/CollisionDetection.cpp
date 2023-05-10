#include "CollisionDetection.h"
#include "../Object/Road/CircleAndLine.h"

CollisionDetection::CollisionDetection()
{
}

CollisionDetection::CollisionDetection(int aIntersectionOneWayLanesNum, double aLaneWidth, BPointCoordinate aIntersectionCenter)
{
	CollisionDetection::IntersectionOneWayLanesNum = aIntersectionOneWayLanesNum;
	CollisionDetection::LaneWidth = aLaneWidth;
	CollisionDetection::IntersectionCenter = aIntersectionCenter;
}

CollisionDetection::~CollisionDetection()
{
}
bool CollisionDetection::IsTwoVehicleCollision(Vehicle aVehicleA, Vehicle aVehicleB)
{
	bool IsCollision = false;

	bool IsCollisionArray[4];

	double deltaWidth = 0.5 * CollisionDetection::LaneWidth;

	if (aVehicleA.SteeringType == 0)
	{
		if (aVehicleB.SteeringType == 0)
		{
			IsCollision = DoesLineIntersectIine(aVehicleA.EnteringNode.Location, aVehicleA.TargetNode.Location, aVehicleB.EnteringNode.Location, aVehicleB.TargetNode.Location);
		}
		else
		{
			IsCollision = DoseCircleAnnualAndLineAnnulaIntersect(aVehicleB.SteeringCenter, aVehicleB.SteeringRadius, aVehicleA.EnteringNode.Location, aVehicleA.TargetNode.Location, CollisionDetection::LaneWidth);
		}
	}
	else
	{
		if (aVehicleB.SteeringType == 0)
		{
			IsCollision = DoseCircleAnnualAndLineAnnulaIntersect(aVehicleA.SteeringCenter, aVehicleA.SteeringRadius, aVehicleB.EnteringNode.Location, aVehicleB.TargetNode.Location, CollisionDetection::LaneWidth);
		}
		else
		{
			IsCollision = DoesTwoCircleAnnularIntersect(aVehicleA.SteeringCenter, aVehicleA.SteeringRadius, aVehicleB.SteeringCenter, aVehicleB.SteeringRadius, CollisionDetection::LaneWidth);
		}
	}
	return IsCollision;
}