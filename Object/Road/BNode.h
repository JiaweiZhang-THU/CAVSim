#ifndef _BNode_
#define _BNode_

#pragma once
#include "BItem.h"
#include<string>


class BNode: public BItem
{
public: 
	BNode(void);
	BNode(double aX, double aY);
	BNode(double aX, double aY, double aDirection);

	BPointCoordinate Location;

	double Direction;

	string GetName(void);

	int RoadBlockType;

	int IntersectionZoneType;

	double GetDistance(BNode point);

};

#endif // !_BNode_