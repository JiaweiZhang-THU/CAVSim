#ifndef _LaneUnit_
#define _LaneUnit_

#pragma once
#include<list>
#include<iostream>
#include<string>

#include "BLine.h"
#include "BRectangle.h"
#include "..\Vehicle\Vehicle.h"

class BConflictSubzone;

class LaneUnit: public BLine
{
public:

	double Width;

	double Length;

	double Direction;

	BPointCoordinate Location;

	int BlockId;

	int FromId;

	int ToId;

	LaneUnit* AdjacentUnitPointer[9];

	BConflictSubzone* BConflictSubzonePointer;

	int MaxVehiclesCanBear;

	int NumberOfVehiclesEnteringOverIntervalTime;

	list<Vehicle> VehicleList;

	void ClearVehicle();

	void UpdateVehicle(Vehicle aVehicle);

	void AddVehicle(Vehicle aVehicle);

	void RemoveVehicle(Vehicle aVehicle);

	void SetFromAndToNode(BNode aFromNode, BNode aToNode);

	string GetName();

	bool IsInside(BNode aBNodePositon);

	LaneUnit();
	LaneUnit(BNode aFromNode, BNode aToNode);
	LaneUnit(int aId, double aWidth, double aLength, BNode aFromNode, BNode aToNode);
	LaneUnit(double aLength);
};

#endif // !_LaneUnit_