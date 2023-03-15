#ifndef _StraightLaneBlock_
#define _StraightLaneBlock_
#pragma once
#include "BNode.h"
#include "BLine.h"
#include "LaneUnit.h"

class StraightLaneBlock: public BNode, public BLine
{
public:
	int Id;

	int LanesNumber;

	double DirectionAngle;

	double StraightBlockLength;

	list<LaneUnit> StraightMultiLanes;

	int NumberOfVehiclesEnteringOverIntervalTime;

	void UpdateNumberOfVehiclesEnteringOverIntervalTime();

	double TrafficFlowRate;

	double GetTrafficFlowRate(double aTimeInterval);

	BRectangle StraightRectangle;

	void UpdateStraightRectangle();

	StraightLaneBlock();

	StraightLaneBlock(int aId, double aDirectionAngle);

	StraightLaneBlock(int aId, BNode aFromNode, BNode aToNode);

	StraightLaneBlock(int aId, int aLanesNumber, BNode aFromNode, BNode aToNode);

	StraightLaneBlock(int aId, int aLanesNumber, BPointCoordinate aLocation);

	StraightLaneBlock(int aId, int aLanesNumber,double aDirectionAngle);

	StraightLaneBlock(int aId, int aLanesNumber, double aDirectionAngle,double aStraightBlockLength);

	StraightLaneBlock(int aId,int aLanesNumber, BPointCoordinate aLocation, BNode aFromNode, BNode aToNode);

	void SetDirectionAngle();

	void SetFromAndToNode();

	string GetName();

	void SetForwardAndReverseStraightLanes();

	void SetLocation(BPointCoordinate aLocation);

private:
	void SetUpdateLaneUnitWithUpdatedLocation();
};

#endif // !_StraightLaneBlock_