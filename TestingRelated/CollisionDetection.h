#ifndef _COLLISION_DETECTION_
#define _COLLISION_DETECTION_

#include "../Object/Road/BPointCoordinate.h"
#include "../Object/Vehicle/Vehicle.h"

class CollisionDetection
{
public:
	int IntersectionOneWayLanesNum;

	double LaneWidth;
	BPointCoordinate IntersectionCenter; 

	bool IsTwoVehicleCollision(Vehicle aVehicleA, Vehicle aVehicleB);
public:
	CollisionDetection();
	CollisionDetection(int aIntersectionOneWayLanesNum, double aLaneWidth, BPointCoordinate aIntersectionCenter);
	~CollisionDetection();

private:

};

#endif // !_COLLISION_DETECTION_
