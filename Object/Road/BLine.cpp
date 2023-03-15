#include "BLine.h"

string BLine::GetName(void)
{
	return "BLineName: Id:" + std::to_string(BLine::Id) + "; Name:" + BLine::Name + " ; From: " + std::to_string(BLine::From_Node.Id) +" to: " + std::to_string(BLine::To_Node.Id);
}

double BLine::GetLength(void)
{
	return BLine::From_Node.GetDistance(BLine::To_Node);
}

bool BLine::Contains(BNode point)
{
	double transvection = (point.Location.X - BLine::To_Node.Location.X) * (BLine::To_Node.Location.X - BLine::From_Node.Location.X) + (point.Location.Y - BLine::To_Node.Location.Y) * (BLine::To_Node.Location.Y - BLine::From_Node.Location.Y);
	return (transvection < 0);
}

double BLine::GetContains(BNode point)
{
	double transvection = (point.Location.X - BLine::To_Node.Location.X) * (BLine::To_Node.Location.X - BLine::From_Node.Location.X) + (point.Location.Y - BLine::To_Node.Location.Y) * (BLine::To_Node.Location.Y - BLine::From_Node.Location.Y);
	return transvection;
}

double BLine::GetArc(void)
{
	int needExtraPi = 0;
	if (BLine::To_Node.Location.Y > BLine::From_Node.Location.Y)
	{
		needExtraPi = 1;
	}
	double deltaX = BLine::To_Node.Location.X - BLine::From_Node.Location.X;
	double line_arc = acos(deltaX / BLine::GetLength()) + needExtraPi * acos(-1); 
	return line_arc;
}

BLine::BLine(void)
{
	BLine::From_Node = BNode();
	BLine::To_Node = BNode();
	
}