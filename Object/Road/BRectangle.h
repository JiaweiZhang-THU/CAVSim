#ifndef _BRECTANGLE_
#define _BRECTANGLE_

#include "BNode.h"

#pragma once
class BRectangle: public BNode
{
public:
	double Width;
	double Length;
	BPointCoordinate Vertex[4];
public:
	BRectangle();
	BRectangle(double aWidth, double aLenght, BPointCoordinate aLocation, double aDirection);
	
	BPointCoordinate RotationCoordiante(BPointCoordinate aOriginalCoordinates, double aTheta);

	void UpdateVertex(BPointCoordinate aLocation,double aNewDriection);

	void InitialVertex();

	bool IsPointInRect(double aX, double aY);

	bool IsIntersect(BRectangle aAnotherRectangle);
};

#endif // !_BRECANGLE_