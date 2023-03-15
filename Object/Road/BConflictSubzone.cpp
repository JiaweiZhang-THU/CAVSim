#include "BConflictSubzone.h"

BConflictSubzone::BConflictSubzone()
{
	BConflictSubzone::SideLength = 4;
	BConflictSubzone::Width = 4;
	BConflictSubzone::Length = 4;

	BConflictSubzone::Location.SetXAndY(0, 0);
	BConflictSubzone::Direction = 0;

	BConflictSubzone::InitialVertex();

	BConflictSubzone::IsDriveInConflictZone = false;
	BConflictSubzone::DriveInLaneUnitPointer = NULL;
	BConflictSubzone::IsDriveOutConflictZone = false;
	BConflictSubzone::DriveOutLaneUnitPointer = NULL;

	for (int i = 0; i < 9; i++)
	{
		BConflictSubzone::AdjacentConflictSubzonePointer[i] = NULL;
	}
}

BConflictSubzone::BConflictSubzone(double aSideLength, BPointCoordinate aLocation, double aDirection)
{
	BConflictSubzone::SideLength = aSideLength;
	BConflictSubzone::Width = aSideLength;
	BConflictSubzone::Length = aSideLength;

	BConflictSubzone::Location = aLocation;
	BConflictSubzone::Direction = aDirection;

	BConflictSubzone::InitialVertex();

	BConflictSubzone::IsDriveInConflictZone = false;
	BConflictSubzone::DriveInLaneUnitPointer = NULL;
	BConflictSubzone::IsDriveOutConflictZone = false;
	BConflictSubzone::DriveOutLaneUnitPointer = NULL;

	for (int i = 0; i < 9; i++)
	{
		BConflictSubzone::AdjacentConflictSubzonePointer[i] = NULL;
	}

}