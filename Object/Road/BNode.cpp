#include "BNode.h"

string BNode::GetName(void)
{
	return "Name:" + BNode::Name + " ;Id:" + std::to_string(BNode::Id) + " ;X:" + std::to_string(BNode::Location.X) + " ;Y:" + std::to_string(BNode::Location.Y) + " ;Direction:" + std::to_string(BNode::Direction);
}

double BNode::GetDistance(BNode point)
{
	return BNode::Location.GetDistance(BNode::Location, point.Location);
}

BNode::BNode(void)
{
	BNode::Id = -1;
	BNode::Location = *new BPointCoordinate();
	BNode::Location.X = 0;
	BNode::Location.Y = 0;
	BNode::Direction = -1;

	BNode::RoadBlockType = 0;
	BNode::IntersectionZoneType = -1;
}

BNode::BNode(double aX, double aY)
{
	BNode::Id = -1;
	BNode::Location = *new BPointCoordinate();
	BNode::Location.X = aX;
	BNode::Location.Y = aY;
	BNode::Direction = -1;

	BNode::RoadBlockType = 0;
	BNode::IntersectionZoneType = -1;
}

BNode::BNode(double aX, double aY, double aDirection)
{
	BNode::Id = -1;
	BNode::Location = *new BPointCoordinate();
	BNode::Location.X = aX;
	BNode::Location.Y = aY;
	BNode::Direction = aDirection;

	BNode::RoadBlockType = 0;
	BNode::IntersectionZoneType = -1;
}