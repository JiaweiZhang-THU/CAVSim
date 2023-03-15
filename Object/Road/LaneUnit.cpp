#include "LaneUnit.h"
#include "../Vehicle/Vehicle.h"
#include "LaneUnitParameters.h"

void LaneUnit::ClearVehicle()
{
	LaneUnit::VehicleList.clear();
}

void LaneUnit::UpdateVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = LaneUnit::VehicleList.begin(); localVehicleIterator != LaneUnit::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			*localVehicleIterator = aVehicle;
		}
	}
}

void LaneUnit::AddVehicle(Vehicle aVehicle)
{
	LaneUnit::VehicleList.push_back(aVehicle);

	if (LaneUnit::Id != aVehicle.LaneId)
	{
		LaneUnit::NumberOfVehiclesEnteringOverIntervalTime += 1;
	}
	
}

void LaneUnit::RemoveVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = LaneUnit::VehicleList.begin(); localVehicleIterator != LaneUnit::VehicleList.end(); localVehicleIterator++) 
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			LaneUnit::VehicleList.erase(localVehicleIterator);
			break;
		}
	}
}

void LaneUnit::SetFromAndToNode(BNode aFromNode, BNode aToNode)
{
	LaneUnit::From_Node = aFromNode;
	LaneUnit::To_Node = aToNode;

	LaneUnit::Location.X = 0.5 * (aFromNode.Location.X + aToNode.Location.X);
	LaneUnit::Location.Y = 0.5 * (aFromNode.Location.Y + aToNode.Location.Y);
}

string LaneUnit::GetName(void)
{
	return "LaneUnit Id:" + std::to_string(LaneUnit::Id) + ";\t FromNode:(X:"  + std::to_string(LaneUnit::From_Node.Location.X) + " ,Y:" + std::to_string(LaneUnit::From_Node.Location.Y) + ") \t" + " ToNode:(X:"  + std::to_string(LaneUnit::To_Node.Location.X) + " ,Y:" + std::to_string(LaneUnit::To_Node.Location.Y) + ") \n";
}

bool LaneUnit::IsInside(BNode aBNodePosition)
{
	BRectangle LaneUnitRectangle(LaneUnit::Width, LaneUnit::Length, LaneUnit::Location, LaneUnit::Direction);
	return LaneUnitRectangle.IsPointInRect(aBNodePosition.Location.X, aBNodePosition.Location.Y);
}

LaneUnit::LaneUnit(void)
{
	LaneUnit::Id     = -1;
	LaneUnit::Width  = paraLaneUnitWidth;
	LaneUnit::Length = paraLaneUnitLength;

	LaneUnit::Direction = 0;

	LaneUnit::FromId = -1;
	LaneUnit::ToId = -1;

	for (int i = 0; i <9; i++)
	{
		LaneUnit::AdjacentUnitPointer[i] = NULL;
	}

	LaneUnit::NumberOfVehiclesEnteringOverIntervalTime = 0;

	LaneUnit::BConflictSubzonePointer = NULL;

	LaneUnit::MaxVehiclesCanBear = int(LaneUnit::Length / paraTwoVehiclesDistance);
}

LaneUnit::LaneUnit(BNode aFromNode, BNode aToNode)
{
	LaneUnit::Id = -1;
	LaneUnit::Width = paraLaneUnitWidth;
	LaneUnit::Length = paraLaneUnitLength;

	LaneUnit::Direction = 0;

	LaneUnit::FromId = -1;
	LaneUnit::ToId = -1;

	LaneUnit::From_Node = aFromNode;
	LaneUnit::To_Node   = aToNode;

	for (int i = 0; i < 9; i++)
	{
		LaneUnit::AdjacentUnitPointer[i] = NULL;
	}

	LaneUnit::NumberOfVehiclesEnteringOverIntervalTime = 0;

	LaneUnit::BConflictSubzonePointer = NULL;

	LaneUnit::MaxVehiclesCanBear = int(LaneUnit::Length / paraTwoVehiclesDistance);
}

LaneUnit::LaneUnit(int aId,double aWidth, double aLength, BNode aFromNode, BNode aToNode)
{
	LaneUnit::Id        = aId;
	LaneUnit::Width     = aWidth;
	LaneUnit::Length    = aLength;

	LaneUnit::Direction = 0;

	LaneUnit::FromId = -1;
	LaneUnit::ToId = -1;
	
	LaneUnit::From_Node = aFromNode;
	LaneUnit::To_Node   = aToNode;

	for (int i = 0; i < 9; i++)
	{
		LaneUnit::AdjacentUnitPointer[i] = NULL;
	}

	LaneUnit::NumberOfVehiclesEnteringOverIntervalTime = 0;

	LaneUnit::BConflictSubzonePointer = NULL;

	LaneUnit::MaxVehiclesCanBear = int(LaneUnit::Length / paraTwoVehiclesDistance);
}

LaneUnit::LaneUnit(double aLength)
{
	LaneUnit::Id = -1;
	LaneUnit::Width = paraLaneUnitWidth;
	LaneUnit::Length = aLength;

	LaneUnit::Direction = 0;

	LaneUnit::FromId = -1;
	LaneUnit::ToId = -1;

	for (int i = 0; i < 9; i++)
	{
		LaneUnit::AdjacentUnitPointer[i] = NULL;
	}

	LaneUnit::NumberOfVehiclesEnteringOverIntervalTime = 0;

	LaneUnit::BConflictSubzonePointer = NULL;

	LaneUnit::MaxVehiclesCanBear = int(LaneUnit::Length / paraTwoVehiclesDistance);
}