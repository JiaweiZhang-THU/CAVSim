#include "Intersection.h"
#include<list>
#include "IntersectionParameters.h"

Intersection::Intersection()
{
	Intersection::NumOfOneWayLanes = paraDefaultNumOfOneWayLanes;
	Intersection::LaneWidth = paraLaneWidthDefinedIndLaneUnit;
	Intersection::IntersectionStraightBlockLength = paraStraightInterfaceLengthDefinedInLaneUnit;

	Intersection::Location = paraDefaultIntersectionLocation;
	Intersection::Direction = paraDefaultDirection;

	Intersection::Length = 2 * (double)Intersection::NumOfOneWayLanes * Intersection::LaneWidth;
	Intersection::Width = Intersection::Length;

	Intersection::NumOfConflictSubzone = (2 * Intersection::NumOfOneWayLanes) * 2 * Intersection::NumOfOneWayLanes;

	Intersection::SetConflictSubzone();
	Intersection::SetConflictSubzoneNet();
	Intersection::SetDriveInAndOutBool();

	Intersection::SetStraightLaneInterface();

	Intersection::SetIntersectionNet();

	Intersection::SetIntersectionNineRectangle();

	Intersection::SetEnteringVehicleInitialPositonAndBoseList();

	Intersection::CurrentMovingInIntersection = 0;
	for (int i = 0; i < 8; i++)
	{
		Intersection::StraightLaneVehiclesRealTimeNum[i] = 0;
	}
	Intersection::InitialStatisticalVariable();
	
}


Intersection::Intersection(int aNumOfOneWayLanes, BPointCoordinate aLocation, double aDirection)
{
	Intersection::NumOfOneWayLanes = aNumOfOneWayLanes;
	Intersection::LaneWidth = paraLaneWidthDefinedIndLaneUnit;
	Intersection::IntersectionStraightBlockLength = paraStraightInterfaceLengthDefinedInLaneUnit;

	Intersection::Location = aLocation;
	Intersection::Direction = aDirection;

	Intersection::Length = 2 * (double)Intersection::NumOfOneWayLanes * Intersection::LaneWidth;
	Intersection::Width = Intersection::Length;

	Intersection::NumOfConflictSubzone = (2 * Intersection::NumOfOneWayLanes) * 2 * Intersection::NumOfOneWayLanes;

	Intersection::SetConflictSubzone();
	Intersection::SetConflictSubzoneNet();
	Intersection::SetDriveInAndOutBool();

	Intersection::SetStraightLaneInterface();

	Intersection::SetIntersectionNet();

	Intersection::SetIntersectionNineRectangle();

	Intersection::SetEnteringVehicleInitialPositonAndBoseList();

	Intersection::CurrentMovingInIntersection = 0;
	for (int i = 0; i < 8; i++)
	{
		Intersection::StraightLaneVehiclesRealTimeNum[i] = 0;
	}
	Intersection::InitialStatisticalVariable();

}

Intersection::Intersection(int aNumOfOneWayLanes, BPointCoordinate aLocation, double aDirection, double aIntersectionStraightBlockLength)
{
	Intersection::NumOfOneWayLanes = aNumOfOneWayLanes;
	Intersection::LaneWidth = paraLaneWidthDefinedIndLaneUnit;
	Intersection::IntersectionStraightBlockLength = aIntersectionStraightBlockLength;

	Intersection::Location = aLocation;
	Intersection::Direction = aDirection;

	Intersection::Length = 2 * (double)Intersection::NumOfOneWayLanes * Intersection::LaneWidth;
	Intersection::Width = Intersection::Length;

	Intersection::NumOfConflictSubzone = (2 * Intersection::NumOfOneWayLanes) * 2 * Intersection::NumOfOneWayLanes;

	Intersection::SetConflictSubzone();
	Intersection::SetConflictSubzoneNet();
	Intersection::SetDriveInAndOutBool();

	Intersection::SetStraightLaneInterface();

	Intersection::SetIntersectionNet();

	Intersection::SetIntersectionNineRectangle();

	Intersection::SetEnteringVehicleInitialPositonAndBoseList();

	Intersection::CurrentMovingInIntersection = 0;
	for (int i = 0; i < 8; i++)
	{
		Intersection::StraightLaneVehiclesRealTimeNum[i] = 0;
	}
	Intersection::InitialStatisticalVariable();

}

void Intersection::SetConflictSubzone()
{
	double localXOrigin = Intersection::Location.X;
	double localYOrigin = Intersection::Location.Y;

	for (int i = 0; i < 2 * Intersection::NumOfOneWayLanes; i++)
	{
		for (int j = 0; j < 2 * Intersection::NumOfOneWayLanes; j++)
		{
			double localX = localXOrigin + ((double)-((double)Intersection::NumOfOneWayLanes - (double)j - 0.5)) * Intersection::LaneWidth;
			double localY = localYOrigin + ((double)((double)Intersection::NumOfOneWayLanes - (double)i - 0.5)) * Intersection::LaneWidth;

			BPointCoordinate localRotatedLocation(localX, localY);
			localRotatedLocation = Intersection::RotationCoordiante(localRotatedLocation, Intersection::Direction);
			
			BConflictSubzone localConflictZone(Intersection::LaneWidth, localRotatedLocation, Intersection::Direction);
			localConflictZone.Id = i * Intersection::NumOfOneWayLanes * 2 + j + 1;
			
			Intersection::ConflictSubzone[i][j] = localConflictZone;

			Intersection::ConflictSubzonesLastPassedVehicleID[i][j] = 0;
			Intersection::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = 0;
		}
	}
}

void Intersection::SetConflictSubzoneNet()
{
	for (int i = 0; i < 2 * Intersection::NumOfOneWayLanes; i++)
	{
		for (int j = 0; j < 2 * Intersection::NumOfOneWayLanes; j++)
		{
			for (int u = -1; u < 2; u++)
			{
				for (int v = -1; v < 2; v++)
				{
					if (i + u >= 0 && i + u < 2 * Intersection::NumOfOneWayLanes && j + v >= 0 && j + v < 2 * Intersection::NumOfOneWayLanes)
					{
						Intersection::ConflictSubzone[i][j].AdjacentConflictSubzonePointer[3 * (u + 1) + v + 1] = &Intersection::ConflictSubzone[i + u][j + v];
					}
					
				}
			}
		}
	}
}


void Intersection::SetDriveInAndOutBool()
{
	for (int i = 0; i < Intersection::NumOfOneWayLanes; i++)
	{
		Intersection::ConflictSubzone[0][i].IsDriveInConflictZone = true;
		Intersection::ConflictSubzone[0][i + Intersection::NumOfOneWayLanes].IsDriveOutConflictZone = true;

		Intersection::ConflictSubzone[i][0].IsDriveOutConflictZone = true;
		Intersection::ConflictSubzone[i + Intersection::NumOfOneWayLanes][0].IsDriveInConflictZone = true;

		Intersection::ConflictSubzone[i][2 * Intersection::NumOfOneWayLanes - 1].IsDriveInConflictZone = true;
		Intersection::ConflictSubzone[i + Intersection::NumOfOneWayLanes][2 * Intersection::NumOfOneWayLanes - 1].IsDriveOutConflictZone = true;

		Intersection::ConflictSubzone[2 * Intersection::NumOfOneWayLanes - 1][i].IsDriveOutConflictZone = true;
		Intersection::ConflictSubzone[2 * Intersection::NumOfOneWayLanes - 1][i + Intersection::NumOfOneWayLanes].IsDriveInConflictZone = true;
	}
}

BPointCoordinate Intersection::RotationCoordiante(BPointCoordinate aOriginalCoordinates, double aTheta)
{
	BPointCoordinate NewCoordinate;
	NewCoordinate.X = aOriginalCoordinates.X * cos(aTheta) - aOriginalCoordinates.Y * sin(aTheta);
	NewCoordinate.Y = aOriginalCoordinates.X * sin(aTheta) + aOriginalCoordinates.Y * cos(aTheta);
	return NewCoordinate;
}


void Intersection::SetStraightLaneInterface()
{
	StraightLaneBlock localNewBlock1(1, Intersection::NumOfOneWayLanes, 1.5 * PI, Intersection::IntersectionStraightBlockLength);
	Intersection::StraightLaneInterface[0] = localNewBlock1;
	StraightLaneBlock localNewBlock2(2, Intersection::NumOfOneWayLanes, 0.5 * PI, Intersection::IntersectionStraightBlockLength);
	Intersection::StraightLaneInterface[1] = localNewBlock2;

	StraightLaneBlock localNewBlock3(3, Intersection::NumOfOneWayLanes, 1.0 * PI, Intersection::IntersectionStraightBlockLength);
	Intersection::StraightLaneInterface[2] = localNewBlock3;
	StraightLaneBlock localNewBlock4(4, Intersection::NumOfOneWayLanes, 0.0 * PI, Intersection::IntersectionStraightBlockLength);
	Intersection::StraightLaneInterface[3] = localNewBlock4;

	StraightLaneBlock localNewBlock5(5, Intersection::NumOfOneWayLanes, 0.5 * PI, Intersection::IntersectionStraightBlockLength);
	Intersection::StraightLaneInterface[4] = localNewBlock5;
	StraightLaneBlock localNewBlock6(6, Intersection::NumOfOneWayLanes, 1.5 * PI, Intersection::IntersectionStraightBlockLength);
	Intersection::StraightLaneInterface[5] = localNewBlock6;

	StraightLaneBlock localNewBlock7(7, Intersection::NumOfOneWayLanes, 0.0 * PI, Intersection::IntersectionStraightBlockLength);
	Intersection::StraightLaneInterface[6] = localNewBlock7;
	StraightLaneBlock localNewBlock8(8, Intersection::NumOfOneWayLanes, 1.0 * PI, Intersection::IntersectionStraightBlockLength);
	Intersection::StraightLaneInterface[7] = localNewBlock8;


	double localBlock1X = - Intersection::NumOfOneWayLanes * paraLaneWidthDefinedIndLaneUnit;
	double localBlock1Y = + Intersection::NumOfOneWayLanes * paraLaneWidthDefinedIndLaneUnit + 0.5 * Intersection::IntersectionStraightBlockLength;
	BPointCoordinate localBlock1Location(localBlock1X, localBlock1Y);
	for (int i = 0; i < 4; i++)
	{
		BPointCoordinate localNewCoordinate = Intersection::RotationCoordiante(localBlock1Location, -0.5 * i * PI);
		localNewCoordinate.X += Intersection::Location.X;
		localNewCoordinate.Y += Intersection::Location.Y;

		Intersection::StraightLaneInterface[2 * i].SetLocation(localNewCoordinate);
	}

	double localBlock2X = + Intersection::NumOfOneWayLanes * paraLaneWidthDefinedIndLaneUnit;
	double localBlock2Y = + Intersection::NumOfOneWayLanes * paraLaneWidthDefinedIndLaneUnit + 0.5 * Intersection::IntersectionStraightBlockLength;
	BPointCoordinate localBlock2Location(localBlock2X, localBlock2Y);
	for (int i = 0; i < 4; i++)
	{
		BPointCoordinate localNewCoordinate = Intersection::RotationCoordiante(localBlock2Location, -0.5 * i * PI);
		localNewCoordinate.X += Intersection::Location.X;
		localNewCoordinate.Y += Intersection::Location.Y;

		Intersection::StraightLaneInterface[1 + 2 * i].SetLocation(localNewCoordinate);
	}
}

void Intersection::SetIntersectionNet()
{
	for (int i = 0; i < 8; i++)
	{
		list<LaneUnit>::iterator localLaneUnitInterator = Intersection::StraightLaneInterface[i].StraightMultiLanes.begin();
		for (int j = 0; j < Intersection::NumOfOneWayLanes; j++,localLaneUnitInterator++)
		{
			switch (i)
			{
			case 0:
				Intersection::ConflictSubzone[0][j].DriveInLaneUnitPointer = &*localLaneUnitInterator;
				localLaneUnitInterator->BConflictSubzonePointer = &Intersection::ConflictSubzone[0][j];
				break;
			case 1:
				Intersection::ConflictSubzone[1][j].DriveOutLaneUnitPointer = &*localLaneUnitInterator;
				localLaneUnitInterator->BConflictSubzonePointer = &Intersection::ConflictSubzone[1][j];
				break;

			case 2:
				Intersection::ConflictSubzone[2][j].DriveInLaneUnitPointer = &*localLaneUnitInterator;
				localLaneUnitInterator->BConflictSubzonePointer = &Intersection::ConflictSubzone[2][j];
				break;
			case 3:
				Intersection::ConflictSubzone[3][j].DriveOutLaneUnitPointer = &*localLaneUnitInterator;
				localLaneUnitInterator->BConflictSubzonePointer = &Intersection::ConflictSubzone[3][j];
				break;

			case 4:
				Intersection::ConflictSubzone[4][j].DriveInLaneUnitPointer = &*localLaneUnitInterator;
				localLaneUnitInterator->BConflictSubzonePointer = &Intersection::ConflictSubzone[4][j];
				break;
			case 5:
				Intersection::ConflictSubzone[5][j].DriveOutLaneUnitPointer = &*localLaneUnitInterator;
				localLaneUnitInterator->BConflictSubzonePointer = &Intersection::ConflictSubzone[5][j];
				break;

			case 6:
				Intersection::ConflictSubzone[6][i].DriveInLaneUnitPointer = &*localLaneUnitInterator;
				localLaneUnitInterator->BConflictSubzonePointer = &Intersection::ConflictSubzone[6][j];
				break;
			case 7:
				Intersection::ConflictSubzone[7][i].DriveOutLaneUnitPointer = &*localLaneUnitInterator;
				localLaneUnitInterator->BConflictSubzonePointer = &Intersection::ConflictSubzone[7][j];
				break;

			default:
				break;
			}
		}
	}
}


void Intersection::ClearVehicleInCrossingArea()
{
	Intersection::VehicleListInCrossingArea.clear();
}

void Intersection::UpdateVehicleInCrossingArea(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = Intersection::VehicleListInCrossingArea.begin(); localVehicleIterator != Intersection::VehicleListInCrossingArea.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			*localVehicleIterator = aVehicle;
		}
	}
}

void Intersection::AddVehicleInCrossingArea(Vehicle aVehicle)
{
	Intersection::VehicleListInCrossingArea.push_back(aVehicle);
}

void Intersection::RemoveVehicleInCrossingArea(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = Intersection::VehicleListInCrossingArea.begin(); localVehicleIterator != Intersection::VehicleListInCrossingArea.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			Intersection::VehicleListInCrossingArea.erase(localVehicleIterator);
			break;
		}
	}
}


void Intersection::ClearVehicle()
{
	Intersection::VehicleList.clear();
}

void Intersection::UpdateVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = Intersection::VehicleList.begin(); localVehicleIterator != Intersection::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			*localVehicleIterator = aVehicle;
		}
	}
}

void Intersection::AddVehicle(Vehicle aVehicle)
{
	Intersection::VehicleList.push_back(aVehicle);
	Intersection::CurrentMovingInIntersection++;
}

void Intersection::RemoveVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = Intersection::VehicleList.begin(); localVehicleIterator != Intersection::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			Intersection::VehicleList.erase(localVehicleIterator);
			Intersection::CurrentMovingInIntersection--;
			break;
		}
	}
}


void Intersection::SetIntersectionNineRectangle()
{
	BRectangle localRectangle0(Intersection::Length, Intersection::Length, Intersection::Location, Intersection::Direction);
	Intersection::IntersectionNineRectangle[0] =localRectangle0; 

	double localBlock1X = - Intersection::NumOfOneWayLanes * paraLaneWidthDefinedIndLaneUnit * 0.5;
	double localBlock1Y = + Intersection::NumOfOneWayLanes * paraLaneWidthDefinedIndLaneUnit + 0.5 * Intersection::IntersectionStraightBlockLength;
	BPointCoordinate localOriginalBlock1Location(localBlock1X, localBlock1Y);

	BPointCoordinate localBlock1Location(Intersection::Location.X + localOriginalBlock1Location.X, Intersection::Location.Y + localOriginalBlock1Location.Y);
	BRectangle localRectangle1(
		Intersection::IntersectionStraightBlockLength,
		Intersection::NumOfOneWayLanes * Intersection::LaneWidth, 
		localBlock1Location, 
		Intersection::Direction
	);
	Intersection::IntersectionNineRectangle[1] = localRectangle1;

	BPointCoordinate localOriginalBlock3Location = Intersection::RotationCoordiante(localOriginalBlock1Location, -0.5 * PI);
	BPointCoordinate localBlock3Location(Intersection::Location.X + localOriginalBlock3Location.X, Intersection::Location.Y + localOriginalBlock3Location.Y);
	BRectangle localRectangle3(
		Intersection::NumOfOneWayLanes * Intersection::LaneWidth,
		Intersection::IntersectionStraightBlockLength,
		localBlock3Location,
		Intersection::Direction
	);
	Intersection::IntersectionNineRectangle[3] = localRectangle3;

	BPointCoordinate localOriginalBlock5Location = Intersection::RotationCoordiante(localOriginalBlock1Location, -1.0 * PI);
	BPointCoordinate localBlock5Location(Intersection::Location.X + localOriginalBlock5Location.X, Intersection::Location.Y + localOriginalBlock5Location.Y);
	BRectangle localRectangle5(
		Intersection::IntersectionStraightBlockLength,
		Intersection::NumOfOneWayLanes * Intersection::LaneWidth,
		localBlock5Location,
		Intersection::Direction
	);
	Intersection::IntersectionNineRectangle[5] = localRectangle5;

	BPointCoordinate localOriginalBlock7Location = Intersection::RotationCoordiante(localOriginalBlock1Location, -1.5 * PI);
	BPointCoordinate localBlock7Location(Intersection::Location.X + localOriginalBlock7Location.X, Intersection::Location.Y + localOriginalBlock7Location.Y);
	BRectangle localRectangle7(
		Intersection::NumOfOneWayLanes * Intersection::LaneWidth,
		Intersection::IntersectionStraightBlockLength,
		localBlock7Location,
		Intersection::Direction
	);
	Intersection::IntersectionNineRectangle[7] = localRectangle7;


	
	double localBlock2X = Intersection::NumOfOneWayLanes * paraLaneWidthDefinedIndLaneUnit * 0.5;
	double localBlock2Y = Intersection::NumOfOneWayLanes * paraLaneWidthDefinedIndLaneUnit + 0.5 * Intersection::IntersectionStraightBlockLength;
	BPointCoordinate localOriginalBlock2Location(localBlock2X, localBlock2Y);

	BPointCoordinate localBlock2Location(Intersection::Location.X + localBlock2X, Intersection::Location.Y + localBlock2Y);

	BRectangle localRectangle2(
		Intersection::IntersectionStraightBlockLength,
		Intersection::NumOfOneWayLanes * Intersection::LaneWidth,
		localBlock2Location,
		Intersection::Direction
	);
	Intersection::IntersectionNineRectangle[2] = localRectangle2;

	BPointCoordinate localOriginalBlock4Location = Intersection::RotationCoordiante(localOriginalBlock2Location, -0.5 * PI);
	BPointCoordinate localBlock4Location(Intersection::Location.X + localOriginalBlock4Location.X, Intersection::Location.Y + localOriginalBlock4Location.Y);
	BRectangle localRectangle4(
		Intersection::NumOfOneWayLanes * Intersection::LaneWidth,
		Intersection::IntersectionStraightBlockLength,
		localBlock4Location,
		Intersection::Direction
	);
	Intersection::IntersectionNineRectangle[4] = localRectangle4;

	BPointCoordinate localOriginalBlock6Location = Intersection::RotationCoordiante(localOriginalBlock2Location, -1.0 * PI);
	BPointCoordinate localBlock6Location(Intersection::Location.X + localOriginalBlock6Location.X, Intersection::Location.Y + localOriginalBlock6Location.Y);
	BRectangle localRectangle6(
		Intersection::IntersectionStraightBlockLength,
		Intersection::NumOfOneWayLanes * Intersection::LaneWidth,
		localBlock6Location,
		Intersection::Direction
	);
	Intersection::IntersectionNineRectangle[6] = localRectangle6;

	BPointCoordinate localOriginalBlock8Location = Intersection::RotationCoordiante(localOriginalBlock2Location, -1.5 * PI);
	BPointCoordinate localBlock8Location(Intersection::Location.X + localOriginalBlock8Location.X, Intersection::Location.Y + localOriginalBlock8Location.Y);
	BRectangle localRectangle8(
		Intersection::NumOfOneWayLanes * Intersection::LaneWidth,
		Intersection::IntersectionStraightBlockLength,
		localBlock8Location,
		Intersection::Direction
	);
	Intersection::IntersectionNineRectangle[8] = localRectangle8;
}

bool Intersection::IsInIntersection(BPointCoordinate aLocation)
{
	for (int i = 0; i < 9; i++)
	{
		if (Intersection::IntersectionNineRectangle[i].IsPointInRect(aLocation.X, aLocation.Y) == true)
		{
			return true;
		}
	}
	return false;
}

bool Intersection::IsInIntersection(double aLocationX, double aLocationY)
{
	for (int i = 0; i < 9; i++)
	{
		if (Intersection::IntersectionNineRectangle[i].IsPointInRect(aLocationX, aLocationY) == true)
		{
			return true;
		}
	}
	return false;
}

bool Intersection::IsInConflictZone(double aLocationX, double aLocationY)
{
	if (Intersection::IntersectionNineRectangle[0].IsPointInRect(aLocationX, aLocationY) == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int Intersection::WhichAreaInIntersection(BPointCoordinate aLocation)
{
	for (int i = 0; i < 9; i++)
	{
		if (Intersection::IntersectionNineRectangle[i].IsPointInRect(aLocation.X, aLocation.Y) == true)
		{
			return i;
		}
	}
	return -1;
}


void Intersection::SetEnteringVehicleInitialPositonAndBoseList()
{
	for (int i = 0; i < 4; i++)
	{
		list<LaneUnit>::iterator localLaneUnitIterator = Intersection::StraightLaneInterface[2 * i].StraightMultiLanes.begin();
		for (int j = 0; j < Intersection::NumOfOneWayLanes; j++)
		{
			BNode aPositionAndPose(localLaneUnitIterator->From_Node.Location.X, localLaneUnitIterator->From_Node.Location.Y, Intersection::StraightLaneInterface[2 * i].DirectionAngle);
			aPositionAndPose.Id = j + 1;
			aPositionAndPose.RoadBlockType = 2;
			aPositionAndPose.IntersectionZoneType = 2 * i + 1;
			Intersection::EnteringVehicleInitialPositonAndPoseList.push_back(aPositionAndPose);

			localLaneUnitIterator++;
		}
	}
}

LaneUnit* Intersection::FindLaneUnitPointer(int aBlockId, int aLaneUnitId)
{
	list<LaneUnit>::iterator localLaneUnitIterator = Intersection::StraightLaneInterface[(int)(aBlockId-1)].StraightMultiLanes.begin();
	for (int j = 0; localLaneUnitIterator != Intersection::StraightLaneInterface[(int)(aBlockId - 1)].StraightMultiLanes.end(); localLaneUnitIterator++)
	{
		j += 1;
		if (j == aLaneUnitId)
		{
			return &(*localLaneUnitIterator);
		}
	}
	return NULL;
}

BNode Intersection::DrivingInPlace(int aBlockId, int aLaneUnitId)
{
	list<LaneUnit>::iterator localLaneUnitIterator = Intersection::StraightLaneInterface[(int)(aBlockId - 1)].StraightMultiLanes.begin();
	for (int j = 0; localLaneUnitIterator != Intersection::StraightLaneInterface[(int)(aBlockId - 1)].StraightMultiLanes.end(); localLaneUnitIterator++)
	{
		j += 1;
		if (j == aLaneUnitId)
		{
			return localLaneUnitIterator->From_Node;
		}
	}
	return localLaneUnitIterator->From_Node;
}

BNode Intersection::DrivingOutPlace(int aBlockId, int aLaneUnitId)
{
	list<LaneUnit>::iterator localLaneUnitIterator = Intersection::StraightLaneInterface[(int)(aBlockId - 1)].StraightMultiLanes.begin();
	for (int j = 0; localLaneUnitIterator != Intersection::StraightLaneInterface[(int)(aBlockId - 1)].StraightMultiLanes.end(); localLaneUnitIterator++)
	{
		j += 1;
		if (j == aLaneUnitId)
		{
			return localLaneUnitIterator->To_Node;
		}
	}
	return localLaneUnitIterator->To_Node;
}

int Intersection::SteerTargetBlock(Vehicle aVehicle, int aSteerType)
{
	int localTargetBlock = 0;
	switch (aSteerType)
	{
	case 0:
		if (aVehicle.BlockId == 1)
		{
			localTargetBlock = 6;
		}
		else
		{
			if (aVehicle.BlockId == 3)
			{
				localTargetBlock = 8;
			}
			else
			{
				if (aVehicle.BlockId == 5)
				{
					localTargetBlock = 2;
				}
				else
				{
					if (aVehicle.BlockId == 7)
					{
						localTargetBlock = 4;
					}
				}
			}
		}
		break;

	case -1:
		if (aVehicle.BlockId == 1)
		{
			localTargetBlock = 4;
		}
		else
		{
			if (aVehicle.BlockId == 3)
			{
				localTargetBlock = 6;
			}
			else
			{
				if (aVehicle.BlockId == 5)
				{
					localTargetBlock = 8;
				}
				else
				{
					if (aVehicle.BlockId == 7)
					{
						localTargetBlock = 2;
					}
				}
			}
		}
		break;

	case 1:
		if (aVehicle.BlockId == 1)
		{
			localTargetBlock = 8;
		}
		else
		{
			if (aVehicle.BlockId == 3)
			{
				localTargetBlock = 2;
			}
			else
			{
				if (aVehicle.BlockId == 5)
				{
					localTargetBlock = 4;
				}
				else
				{
					if (aVehicle.BlockId == 7)
					{
						localTargetBlock = 6;
					}
				}
			}
		}
		break;

	default:
		break;
	}

	return localTargetBlock;
}

string Intersection::GetName()
{
	string Name = to_string(Intersection::Id) + " Intersection Lane :\n";

	return Name;
}

BConflictSubzone Intersection::FindConflictSubzone(double aX, double aY)
{
	double localDeltaX = aX - (Intersection::ConflictSubzone[0][0].Location.X - 0.5 * Intersection::LaneWidth);
	double localDeltaY = -(aY - (Intersection::ConflictSubzone[0][0].Location.Y + 0.5 * Intersection::LaneWidth));

	int localXIndex = int((int)localDeltaX / Intersection::LaneWidth);
	int localYIndex = int((int)localDeltaY / Intersection::LaneWidth);

	if (localXIndex < 0 || localXIndex>2 * Intersection::NumOfOneWayLanes - 1 || localYIndex < 0 || localYIndex>2 * Intersection::NumOfOneWayLanes - 1)
	{
		cout << "[Intersection::FindConflictSubzone] ERROR: overstep the boundary" << endl;
		return Intersection::ConflictSubzone[localYIndex][localXIndex];
	}
	else
	{
		return Intersection::ConflictSubzone[localYIndex][localXIndex];
	}
}

void Intersection::VehicleTrajectoryCoverageConflictSubzones(Vehicle& aVehicle)
{
	BPointCoordinate localStartingLocation = aVehicle.EnteringNode.Location;
	BPointCoordinate localTargetLocation = aVehicle.TargetNode.Location;
	int localBPCSet[40];
	int num = 0;
	double deltaTheta = (acos(-1) / 2) / ((double)paraSamplingNumOnTrajectory + 1);

	double BasicTheta = 0;
	while (aVehicle.SteeringType!=0)
	{
		double localX = aVehicle.SteeringCenter.X + aVehicle.SteeringRadius * cos(BasicTheta + 1 * deltaTheta);
		double localY = aVehicle.SteeringCenter.Y + aVehicle.SteeringRadius * sin(BasicTheta + 1 * deltaTheta);
		if (Intersection::IsInConflictZone(localX, localY) == false)
		{
			BasicTheta += acos(-1) / 2;
		}
		else
		{
			break;
		}
	}

	for (int i = 1; i < paraSamplingNumOnTrajectory + 1; i++)
	{
		if (aVehicle.SteeringType != 0)
		{
			double localX = aVehicle.SteeringCenter.X + aVehicle.SteeringRadius * cos(BasicTheta + i * deltaTheta);
			double localY = aVehicle.SteeringCenter.Y + aVehicle.SteeringRadius * sin(BasicTheta + i * deltaTheta);
			BConflictSubzone localConflictSubzone = Intersection::FindConflictSubzone(
				localX,
				localY
			);

			if (num == 0 || localBPCSet[num - 1] != localConflictSubzone.Id)
			{
				localBPCSet[num] = localConflictSubzone.Id;
				num += 1;
			}

		}
		else
		{
			BConflictSubzone localConflictSubzone = Intersection::FindConflictSubzone(
				localStartingLocation.X + i * (localTargetLocation.X - localStartingLocation.X) / ((double)paraSamplingNumOnTrajectory + 1),
				localStartingLocation.Y + i * (localTargetLocation.Y - localStartingLocation.Y) / ((double)paraSamplingNumOnTrajectory + 1)
			);

			if (num == 0 || localBPCSet[num - 1] != localConflictSubzone.Id)
			{
				localBPCSet[num] = localConflictSubzone.Id;
				num += 1;
			}
		}
	}

	aVehicle.TrajectoryCoverageConflictSubzonesArraySize = num;
	for (int i=0; i<num; i++)
	{
		aVehicle.TrajectoryCoverageConflictSubzonesArray[i] = localBPCSet[i];
	}
}

double Intersection::GetSteerRadius(int aLaneId, int aSteeringType)
{
	double localSteerRadius;
	if (aSteeringType == -1)
	{
		localSteerRadius = (2 * Intersection::NumOfOneWayLanes - aLaneId + 0.5) * Intersection::LaneWidth;
	}
	else
	{
		if (aSteeringType == 1)
		{
			localSteerRadius = (aLaneId - 0.5) * Intersection::LaneWidth;
		}
		else
		{
			localSteerRadius = 0;
		}
	}
	return localSteerRadius;
}

BPointCoordinate Intersection::GetSteerCenter(int aBlockId, int aSteeringType)
{
	if (aSteeringType == -1)
	{
		switch (aBlockId)
		{
		case 1:
			return Intersection::IntersectionNineRectangle[0].Vertex[0];
			break;
		case 3:
			return Intersection::IntersectionNineRectangle[0].Vertex[3];
			break;
		case 5:
			return Intersection::IntersectionNineRectangle[0].Vertex[2];
			break;
		case 7:
			return Intersection::IntersectionNineRectangle[0].Vertex[1];
			break;
		default:
			break;
		}
	}
	else
	{
		if (aSteeringType == 1)
		{
			switch (aBlockId)
			{
			case 1:
				return Intersection::IntersectionNineRectangle[0].Vertex[1];
				break;
			case 3:
				return Intersection::IntersectionNineRectangle[0].Vertex[0];
				break;
			case 5:
				return Intersection::IntersectionNineRectangle[0].Vertex[3];
				break;
			case 7:
				return Intersection::IntersectionNineRectangle[0].Vertex[2];
				break;
			default:
				break;
			}
		}
	}
	return Intersection::IntersectionNineRectangle[0].Vertex[0];
}

void Intersection::InitialStatisticalVariable()
{
	for (int i = 0; i < 8; i++)
	{
		Intersection::StraightLaneVehiclesAN[i] = 0;
		Intersection::StraightLaneVehiclesAnExiting[i] = 0;
	}
	Intersection::PassingIntersectionVehiclesNum = 0;
	
}

TurnDirectionType Intersection::WhichTurnDirectionType(int EnteringBlockId, int TargetBlockId)
{
	if (EnteringBlockId == 1)
	{
		switch (TargetBlockId)
		{
		case 8:
			return TurnDirectionType::NSRight;
			break;
		case 6:
			return TurnDirectionType::NSStraight;
			break;
		case 4:
			return TurnDirectionType::NSLeft;
			break;
		default:
			break;
		}
	}
	
	if (EnteringBlockId == 3)
	{
		switch (TargetBlockId)
		{
		case 2:
			return TurnDirectionType::WERight;
			break;
		case 8:
			return TurnDirectionType::WEStraight;
			break;
		case 6:
			return TurnDirectionType::WELeft;
			break;
		default:
			break;
		}
	}

	if (EnteringBlockId == 5)
	{
		switch (TargetBlockId)
		{
		case 4:
			return TurnDirectionType::NSRight;
			break;
		case 2:
			return TurnDirectionType::NSStraight;
			break;
		case 8:
			return TurnDirectionType::NSLeft;
			break;
		default:
			break;
		}
	}

	if (EnteringBlockId == 7)
	{
		switch (TargetBlockId)
		{
		case 6:
			return TurnDirectionType::WERight;
			break;
		case 4:
			return TurnDirectionType::WEStraight;
			break;
		case 2:
			return TurnDirectionType::WELeft;
			break;
		default:
			break;
		}
	}
}