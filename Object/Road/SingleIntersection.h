#ifndef _SINGLE_INTERSECTION_
#define _SINGLE_INTERSECTION_

#include "Intersection.h"
#include "StraightLane.h"
#include "..\..\Algorithm\Others\cls_random.h"

class SingleIntersection
{
public:

	cls_random MyRandom;

	Intersection Intersection0;

	double StraightBlockInterfaceLength;

	void SetIntersection();

	SingleIntersection();

	list<BNode> GetEnteringVehicleInitialPositonAndBoseList();

	BNode OriginalRoadNodePassingIntersection(Vehicle aVehicle);

	BNode NextRoadNodePassingIntersection(Vehicle& aVehicle);

	int AssignSteeringType(int aVehicleId,int aLaneId);

	int WhichRoadBlockKind(BPointCoordinate aLocation);

	bool IsInGriddedMap(BPointCoordinate aLocation);
};


#endif // !_SINGLE_INTERSECTION_
