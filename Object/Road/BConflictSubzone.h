#ifndef _BASIC_CONFLICT_SUBZONE_
#define _BASIC_CONFLICT_SUBZONE_

#include "BRectangle.h"
#include "LaneUnit.h"

class BConflictSubzone: public BRectangle
{
public:
	double SideLength;

	bool IsDriveInConflictZone;

	LaneUnit* DriveInLaneUnitPointer;

	bool IsDriveOutConflictZone;

	LaneUnit* DriveOutLaneUnitPointer;
	
	int LastVehcileId;

	double LastVehicleArrivalTime;

	BConflictSubzone();

	BConflictSubzone(double aSideLength, BPointCoordinate aLocation, double aDirection);

	BConflictSubzone* AdjacentConflictSubzonePointer[9];

};


#endif // !_BASIC_CONFLICT_SUBZONE_

