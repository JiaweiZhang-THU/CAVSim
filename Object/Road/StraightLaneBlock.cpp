#include<math.h>
#include<iostream>
#include "StraightLaneBlock.h"
#include "StraightLaneBlockParameters.h"

const double ANGLE_DELTA_NOISE = 0.1;
const double PI = acos(-1);

StraightLaneBlock::StraightLaneBlock()
{
	StraightLaneBlock::Id = -1;
	StraightLaneBlock::LanesNumber = paraDefaultLaneNum;

	BPointCoordinate aLocation(0, 0);
	StraightLaneBlock::Location = aLocation;

	StraightLaneBlock::StraightBlockLength = paraStraightBlockLength;

	BNode aFromNode(StraightLaneBlock::Location.X - paraStraightBlockLength / 2, StraightLaneBlock::Location.Y);
	StraightLaneBlock::From_Node = aFromNode;
	BNode aToNode(StraightLaneBlock::Location.X + paraStraightBlockLength / 2, StraightLaneBlock::Location.Y);
	StraightLaneBlock::To_Node = aToNode;

	StraightLaneBlock::SetDirectionAngle();
	StraightLaneBlock::SetForwardAndReverseStraightLanes();

	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;

	StraightLaneBlock::UpdateStraightRectangle();

}

StraightLaneBlock::StraightLaneBlock(int aId, double aDirectionAngle)
{
	StraightLaneBlock::Id = aId;
	StraightLaneBlock::LanesNumber = paraDefaultLaneNum;
	StraightLaneBlock::DirectionAngle = aDirectionAngle;

	BPointCoordinate aLocation(0, 0);
	StraightLaneBlock::Location = aLocation;

	StraightLaneBlock::StraightBlockLength = paraStraightBlockLength;

	StraightLaneBlock::SetFromAndToNode();
	StraightLaneBlock::SetForwardAndReverseStraightLanes();

	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;

	StraightLaneBlock::UpdateStraightRectangle();
}

StraightLaneBlock::StraightLaneBlock(int aId, BNode aFromNode, BNode aToNode)
{
	StraightLaneBlock::Id = aId;
	StraightLaneBlock::LanesNumber = paraDefaultLaneNum;

	BPointCoordinate aLocation(0, 0);
	StraightLaneBlock::Location = aLocation;

	StraightLaneBlock::StraightBlockLength = paraStraightBlockLength;

	StraightLaneBlock::From_Node = aFromNode;
	StraightLaneBlock::To_Node = aToNode;

	StraightLaneBlock::SetDirectionAngle();
	StraightLaneBlock::SetForwardAndReverseStraightLanes();

	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;

	StraightLaneBlock::UpdateStraightRectangle();
}

StraightLaneBlock::StraightLaneBlock(int aId, int aLanesNumber, BNode aFromNode, BNode aToNode)
{
	StraightLaneBlock::Id = aId;
	StraightLaneBlock::LanesNumber = aLanesNumber;

	BPointCoordinate aLocation(0, 0);
	StraightLaneBlock::Location = aLocation;

	StraightLaneBlock::StraightBlockLength = paraStraightBlockLength;

	StraightLaneBlock::From_Node = aFromNode;
	StraightLaneBlock::To_Node = aToNode;

	StraightLaneBlock::SetDirectionAngle();
	StraightLaneBlock::SetForwardAndReverseStraightLanes();

	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;

	StraightLaneBlock::UpdateStraightRectangle();
}

StraightLaneBlock::StraightLaneBlock(int aId, int aLanesNumber, BPointCoordinate aLocation)
{
	StraightLaneBlock::Id = aId;
	StraightLaneBlock::LanesNumber = aLanesNumber;
	StraightLaneBlock::Location = aLocation;

	StraightLaneBlock::StraightBlockLength = paraStraightBlockLength;

	BNode aFromNode(StraightLaneBlock::Location.X - paraStraightBlockLength / 2, StraightLaneBlock::Location.Y);
	StraightLaneBlock::From_Node = aFromNode;
	BNode aToNode(StraightLaneBlock::Location.X + paraStraightBlockLength / 2, StraightLaneBlock::Location.Y);
	StraightLaneBlock::To_Node = aToNode;

	StraightLaneBlock::SetDirectionAngle();
	StraightLaneBlock::SetForwardAndReverseStraightLanes();

	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;

	StraightLaneBlock::UpdateStraightRectangle();
}

StraightLaneBlock::StraightLaneBlock(int aId, int aLanesNumber, double aDirectionAngle)
{
	StraightLaneBlock::Id = aId;
	StraightLaneBlock::LanesNumber = aLanesNumber;
	StraightLaneBlock::Direction = aDirectionAngle;
	StraightLaneBlock::DirectionAngle = aDirectionAngle;

	BPointCoordinate aLocation(0, 0);
	StraightLaneBlock::Location = aLocation;

	StraightLaneBlock::StraightBlockLength = paraStraightBlockLength;

	StraightLaneBlock::SetFromAndToNode();
	StraightLaneBlock::SetForwardAndReverseStraightLanes();

	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;

	StraightLaneBlock::UpdateStraightRectangle();
}

StraightLaneBlock::StraightLaneBlock(int aId, int aLanesNumber, double aDirectionAngle,double aStraightBlockLength)
{
	StraightLaneBlock::Id = aId;
	StraightLaneBlock::LanesNumber = aLanesNumber;
	StraightLaneBlock::Direction = aDirectionAngle;
	StraightLaneBlock::DirectionAngle = aDirectionAngle;

	BPointCoordinate aLocation(0, 0);
	StraightLaneBlock::Location = aLocation;

	StraightLaneBlock::StraightBlockLength = aStraightBlockLength;

	StraightLaneBlock::SetFromAndToNode();
	StraightLaneBlock::SetForwardAndReverseStraightLanes();

	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;

	StraightLaneBlock::UpdateStraightRectangle();
}

StraightLaneBlock::StraightLaneBlock(int aId, int aLanesNumber, BPointCoordinate aLocation, BNode aFromNode, BNode aToNode)
{
	StraightLaneBlock::Id = aId;
	StraightLaneBlock::LanesNumber = aLanesNumber;
	StraightLaneBlock::Location = aLocation;

	StraightLaneBlock::StraightBlockLength = paraStraightBlockLength;

	StraightLaneBlock::From_Node = aFromNode;
	StraightLaneBlock::To_Node = aToNode;

	StraightLaneBlock::SetDirectionAngle();
	StraightLaneBlock::SetForwardAndReverseStraightLanes();

	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;

	StraightLaneBlock::UpdateStraightRectangle();
}

void StraightLaneBlock::SetDirectionAngle(void)
{
	double UnnormalizedDirectionAngle = StraightLaneBlock::GetArc();

	StraightLaneBlock::DirectionAngle = UnnormalizedDirectionAngle;
}

void StraightLaneBlock::SetFromAndToNode(void)
{
	double localXDelta = 0.5 * StraightLaneBlock::StraightBlockLength * cos(StraightLaneBlock::DirectionAngle);
	double localYDelta = 0.5 * StraightLaneBlock::StraightBlockLength * sin(StraightLaneBlock::DirectionAngle);

	BNode aFromNode(StraightLaneBlock::Location.X - localXDelta, StraightLaneBlock::Location.Y - localYDelta);
	StraightLaneBlock::From_Node = aFromNode;
	BNode aToNode(StraightLaneBlock::Location.X + localXDelta, StraightLaneBlock::Location.Y + localYDelta);
	StraightLaneBlock::To_Node = aToNode;
}

void StraightLaneBlock::SetForwardAndReverseStraightLanes()
{
	for (int i = 0; i < StraightLaneBlock::LanesNumber; i++)
	{
		LaneUnit aLaneUnit(StraightLaneBlock::StraightBlockLength);
		aLaneUnit.Id = i + 1;
		aLaneUnit.Direction = StraightLaneBlock::DirectionAngle;
		aLaneUnit.BlockId = StraightLaneBlock::Id;
		aLaneUnit.Length = StraightLaneBlock::StraightBlockLength;
		StraightLaneBlock::StraightMultiLanes.push_back(aLaneUnit);
	}

	list<LaneUnit>::iterator aIterIndex;

	if (StraightLaneBlock::DirectionAngle > 0 - ANGLE_DELTA_NOISE && StraightLaneBlock::DirectionAngle < 0 + ANGLE_DELTA_NOISE)
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.Y += (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			aIterIndex->To_Node.Location.Y += (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
		}

	}

	else if (StraightLaneBlock::DirectionAngle > PI / 2 - ANGLE_DELTA_NOISE && StraightLaneBlock::DirectionAngle < PI / 2 + ANGLE_DELTA_NOISE)
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.X -= (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			aIterIndex->To_Node.Location.X -= (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
		}
	}

	else if (StraightLaneBlock::DirectionAngle > 2 * PI / 2 - ANGLE_DELTA_NOISE && StraightLaneBlock::DirectionAngle < 2 * PI / 2 + ANGLE_DELTA_NOISE)
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.Y -= (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			aIterIndex->To_Node.Location.Y -= (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
		}
	}

	else if (StraightLaneBlock::DirectionAngle > 3 * PI / 2 - ANGLE_DELTA_NOISE && StraightLaneBlock::DirectionAngle < 3 * PI / 2 + ANGLE_DELTA_NOISE)
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.X += (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			aIterIndex->To_Node.Location.X += (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
		}
	}
	else
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.X -= ((i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width) * sin(StraightLaneBlock::Direction);
			aIterIndex->To_Node.Location.X -= ((i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width) * sin(StraightLaneBlock::Direction);
			aIterIndex->From_Node.Location.Y += ((i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width) * cos(StraightLaneBlock::Direction);
			aIterIndex->To_Node.Location.Y += ((i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width) * cos(StraightLaneBlock::Direction);

			aIterIndex->Location.X = 0.5 * (aIterIndex->From_Node.Location.X + aIterIndex->To_Node.Location.X);
			aIterIndex->Location.Y = 0.5 * (aIterIndex->From_Node.Location.Y + aIterIndex->To_Node.Location.Y);
		}
	}

	StraightLaneBlock::UpdateStraightRectangle();
}

void StraightLaneBlock::SetLocation(BPointCoordinate aLocation)
{
	StraightLaneBlock::Location = aLocation;

	double localXDelta = 0.5 * StraightLaneBlock::StraightBlockLength * cos(StraightLaneBlock::DirectionAngle);
	double localYDelta = 0.5 * StraightLaneBlock::StraightBlockLength * sin(StraightLaneBlock::DirectionAngle);

	BNode aFromNode(StraightLaneBlock::Location.X - localXDelta, StraightLaneBlock::Location.Y - localYDelta);
	StraightLaneBlock::From_Node = aFromNode;
	BNode aToNode(StraightLaneBlock::Location.X + localXDelta, StraightLaneBlock::Location.Y + localYDelta);
	StraightLaneBlock::To_Node = aToNode;

	StraightLaneBlock::SetUpdateLaneUnitWithUpdatedLocation();

	StraightLaneBlock::UpdateStraightRectangle();
}

void StraightLaneBlock::SetUpdateLaneUnitWithUpdatedLocation()
{
	list<LaneUnit>::iterator aIterIndex;

	if (StraightLaneBlock::DirectionAngle > 0 - ANGLE_DELTA_NOISE && StraightLaneBlock::DirectionAngle < 0 + ANGLE_DELTA_NOISE)
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.Y += (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			aIterIndex->To_Node.Location.Y += (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			
			aIterIndex->Location.X = 0.5 * (aIterIndex->From_Node.Location.X + aIterIndex->To_Node.Location.X);
			aIterIndex->Location.Y = 0.5 * (aIterIndex->From_Node.Location.Y + aIterIndex->To_Node.Location.Y);

		}
	}

	else if (StraightLaneBlock::DirectionAngle > PI / 2 - ANGLE_DELTA_NOISE && StraightLaneBlock::DirectionAngle < PI / 2 + ANGLE_DELTA_NOISE)
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.X -= (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			aIterIndex->To_Node.Location.X -= (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;

			aIterIndex->Location.X = 0.5 * (aIterIndex->From_Node.Location.X + aIterIndex->To_Node.Location.X);
			aIterIndex->Location.Y = 0.5 * (aIterIndex->From_Node.Location.Y + aIterIndex->To_Node.Location.Y);
		}
	}

	else if (StraightLaneBlock::DirectionAngle > 2 * PI / 2 - ANGLE_DELTA_NOISE && StraightLaneBlock::DirectionAngle < 2 * PI / 2 + ANGLE_DELTA_NOISE)
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.Y -= (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			aIterIndex->To_Node.Location.Y -= (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;

			aIterIndex->Location.X = 0.5 * (aIterIndex->From_Node.Location.X + aIterIndex->To_Node.Location.X);
			aIterIndex->Location.Y = 0.5 * (aIterIndex->From_Node.Location.Y + aIterIndex->To_Node.Location.Y);
		}
	}

	else if (StraightLaneBlock::DirectionAngle > 3 * PI / 2 - ANGLE_DELTA_NOISE && StraightLaneBlock::DirectionAngle < 3 * PI / 2 + ANGLE_DELTA_NOISE)
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.X += (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;
			aIterIndex->To_Node.Location.X += (i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width;

			aIterIndex->Location.X = 0.5 * (aIterIndex->From_Node.Location.X + aIterIndex->To_Node.Location.X);
			aIterIndex->Location.Y = 0.5 * (aIterIndex->From_Node.Location.Y + aIterIndex->To_Node.Location.Y);
		}
	}
	else
	{
		double i = 0;
		for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
		{
			i += 1;
			aIterIndex->From_Node = StraightLaneBlock::From_Node;
			aIterIndex->To_Node = StraightLaneBlock::To_Node;
			aIterIndex->Direction = StraightLaneBlock::Direction;

			aIterIndex->From_Node.Location.X -= ((i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width) * sin(StraightLaneBlock::Direction);
			aIterIndex->To_Node.Location.X -= ((i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width) * sin(StraightLaneBlock::Direction);
			aIterIndex->From_Node.Location.Y += ((i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width) * cos(StraightLaneBlock::Direction);
			aIterIndex->To_Node.Location.Y += ((i - 1.0) * aIterIndex->Width + 0.5 * aIterIndex->Width) * cos(StraightLaneBlock::Direction);

			aIterIndex->Location.X = 0.5 * (aIterIndex->From_Node.Location.X + aIterIndex->To_Node.Location.X);
			aIterIndex->Location.Y = 0.5 * (aIterIndex->From_Node.Location.Y + aIterIndex->To_Node.Location.Y);
		}

	}

	StraightLaneBlock::UpdateStraightRectangle();
}

string StraightLaneBlock::GetName()
{
	string Name = "Straight Block Id: " + std::to_string(StraightLaneBlock::Id)+":\n\tEach Lane:\n";
	list<LaneUnit>::iterator aIterIndex;
	for (aIterIndex = StraightLaneBlock::StraightMultiLanes.begin(); aIterIndex != StraightLaneBlock::StraightMultiLanes.end(); aIterIndex++)
	{
		string LaneUnitName = aIterIndex->GetName();
		Name = Name + "\t" + LaneUnitName;
	}
	return Name;
}

void StraightLaneBlock::UpdateNumberOfVehiclesEnteringOverIntervalTime()
{
	StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime = 0;
	list<LaneUnit>::iterator localLaneUnitIterator;
	for (localLaneUnitIterator = StraightLaneBlock::StraightMultiLanes.begin(); localLaneUnitIterator != StraightLaneBlock::StraightMultiLanes.end(); localLaneUnitIterator++)
	{
		StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime += localLaneUnitIterator->NumberOfVehiclesEnteringOverIntervalTime;
		localLaneUnitIterator->NumberOfVehiclesEnteringOverIntervalTime = 0;
	}

}

double StraightLaneBlock::GetTrafficFlowRate(double aTimeInterval)
{
	StraightLaneBlock::UpdateNumberOfVehiclesEnteringOverIntervalTime();
	StraightLaneBlock::TrafficFlowRate = (double)StraightLaneBlock::NumberOfVehiclesEnteringOverIntervalTime / aTimeInterval;
	
	return StraightLaneBlock::TrafficFlowRate;
}

void StraightLaneBlock::UpdateStraightRectangle()
{
	BRectangle localRoadRectangle(StraightLaneBlock::LanesNumber * paraLaneWidth, StraightLaneBlock::StraightBlockLength, StraightLaneBlock::Location, StraightLaneBlock::Direction);
	StraightLaneBlock::StraightRectangle = localRoadRectangle;
}
