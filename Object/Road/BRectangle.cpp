#include "BRectangle.h"
#include <math.h>

BRectangle::BRectangle()
{
	BRectangle::Width = 0;
	BRectangle::Length = 0;
	BRectangle::Direction = 0;
}

BRectangle::BRectangle(double aWidth, double aLenght, BPointCoordinate aLocation, double aDirection)
{
	BRectangle::Width = aWidth;
	BRectangle::Length = aLenght;
	BRectangle::Location = aLocation;
	BRectangle::Direction = aDirection;

	BRectangle::InitialVertex();
}

BPointCoordinate BRectangle::RotationCoordiante(BPointCoordinate aOriginalCoordinates, double aTheta)
{
	BPointCoordinate NewCoordinate;
	NewCoordinate.X = aOriginalCoordinates.X * cos(aTheta) - aOriginalCoordinates.Y * sin(aTheta);
	NewCoordinate.Y = aOriginalCoordinates.X * sin(aTheta) + aOriginalCoordinates.Y * cos(aTheta);
	return NewCoordinate;
}

void BRectangle::InitialVertex()
{
	BRectangle::Vertex[0].X = 0.5 * BRectangle::Length;
	BRectangle::Vertex[0].Y = 0.5 * BRectangle::Width;

	BRectangle::Vertex[1].X = - 0.5 * BRectangle::Length;
	BRectangle::Vertex[1].Y = 0.5 * BRectangle::Width;

	BRectangle::Vertex[2].X = - 0.5 * BRectangle::Length;
	BRectangle::Vertex[2].Y = - 0.5 * BRectangle::Width;

	BRectangle::Vertex[3].X = 0.5 * BRectangle::Length;
	BRectangle::Vertex[3].Y = - 0.5 * BRectangle::Width;

	for (int i = 0; i < 4; i++)
	{
		BRectangle::Vertex[i] = BRectangle::RotationCoordiante(BRectangle::Vertex[i], BRectangle::Direction);
	}

	for (int i = 0; i < 4; i++)
	{
		BRectangle::Vertex[i].X += BRectangle::Location.X;
		BRectangle::Vertex[i].Y += BRectangle::Location.Y;
	}
}

bool BRectangle::IsPointInRect(double aX, double aY)
{
	BPointCoordinate A = BRectangle::Vertex[0];
	BPointCoordinate B = BRectangle::Vertex[1];
	BPointCoordinate C = BRectangle::Vertex[2];
	BPointCoordinate D = BRectangle::Vertex[3];

	double a = (B.X - A.X) * (aY - A.Y) - (B.Y - A.Y) * (aX - A.X);
	double b = (C.X - B.X) * (aY - B.Y) - (C.Y - B.Y) * (aX - B.X);
	double c = (D.X - C.X) * (aY - C.Y) - (D.Y - C.Y) * (aX - C.X);
	double d = (A.X - D.X) * (aY - D.Y) - (A.Y - D.Y) * (aX - D.X);

	if ((a >= 0 && b >= 0 && c >= 0 && d >= 0) || (a <= 0 && b <= 0 && c <= 0 && d <= 0)) {
		return true;
	}
	return false;
}

void BRectangle::UpdateVertex(BPointCoordinate aLocation,double aNewDriection)
{
	BRectangle::Location = aLocation;
	BRectangle::Direction = aNewDriection;
	BRectangle::InitialVertex();
}

bool BRectangle::IsIntersect(BRectangle aAnotherRectangle)
{
	bool localIsIntersect = false;
	for (int i = 0; i < 4; i++)
	{
		if (BRectangle::IsPointInRect(aAnotherRectangle.Vertex[i].X, aAnotherRectangle.Vertex[i].Y) == true)
		{
			localIsIntersect = true;
			return localIsIntersect;
		}
	}

	for (int i = 0; i < 4; i++)
	{
		if (aAnotherRectangle.IsPointInRect(BRectangle::Vertex[i].X, BRectangle::Vertex[i].Y) == true)
		{
			localIsIntersect = true;
			return localIsIntersect;
		}
	}
	return localIsIntersect;
}