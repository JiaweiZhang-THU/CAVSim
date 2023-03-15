#pragma once
#include<math.h>
#include"BPointCoordinate.h"

bool DoesCircleIntersectIine(BPointCoordinate aCircleCenter, double aRadius, BPointCoordinate APoint, BPointCoordinate BPoint)
{
	double A = APoint.Y - BPoint.Y;
	double B = BPoint.X - APoint.X;
	double C = APoint.X * BPoint.Y - BPoint.X * APoint.Y;

	double distanceBetweenCircleAndLine = fabs(A * aCircleCenter.X + B * aCircleCenter.Y + C) / sqrt(A * A + B * B);
	if (distanceBetweenCircleAndLine > aRadius)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool DoesCircleIntersectCircle(BPointCoordinate aCircleCenter1, double aRadius1, BPointCoordinate aCircleCenter2, double aRadius2)
{
	double deltaX = aCircleCenter1.X - aCircleCenter2.X;
	double deltaY = aCircleCenter1.Y - aCircleCenter2.Y;
	double CenterDistanceBetweenCircleAndCircle = sqrt(deltaX * deltaX + deltaY * deltaY);
	if (CenterDistanceBetweenCircleAndCircle < aRadius1 + aRadius2)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DoesLineIntersectIine(BPointCoordinate A, BPointCoordinate B, BPointCoordinate C, BPointCoordinate D)
{
	
	if (!(fmin(A.X, B.X) <= fmax(C.X, D.X) && fmin(C.Y, D.Y) <= fmax(A.Y, B.Y) && fmin(C.X, D.X) <= fmax(A.X, B.X) && fmin(A.Y, B.Y) <= fmax(C.Y, D.Y)))
		return false;

	double u, v, w, z;
	u = (C.X - A.X) * (B.Y - A.Y) - (B.X - A.X) * (C.Y - A.Y);
	v = (D.X - A.X) * (B.Y - A.Y) - (B.X - A.X) * (D.Y - A.Y);
	w = (A.X - C.X) * (D.Y - C.Y) - (D.X - C.X) * (A.Y - C.Y);
	z = (B.X - C.X) * (D.Y - C.Y) - (D.X - C.X) * (B.Y - C.Y);
	return (u * v <= 0.00000001 && w * z <= 0.00000001);
}

bool DoesTwoCircleAnnularIntersect(BPointCoordinate aCircleCenter1, double aRadius1, BPointCoordinate aCircleCenter2, double aRadius2, double AnnualWidth)
{
	bool IsIntersect1 = DoesCircleIntersectCircle(aCircleCenter1, aRadius1 - 0.5 * AnnualWidth, aCircleCenter2, aRadius2 - 0.5 * AnnualWidth);
	bool IsIntersect2 = DoesCircleIntersectCircle(aCircleCenter1, aRadius1 + 0.5 * AnnualWidth, aCircleCenter2, aRadius2 - 0.5 * AnnualWidth);
	bool IsIntersect3 = DoesCircleIntersectCircle(aCircleCenter1, aRadius1 - 0.5 * AnnualWidth, aCircleCenter2, aRadius2 + 0.5 * AnnualWidth);
	bool IsIntersect4 = DoesCircleIntersectCircle(aCircleCenter1, aRadius1 + 0.5 * AnnualWidth, aCircleCenter2, aRadius2 + 0.5 * AnnualWidth);
	return IsIntersect1 || IsIntersect2 || IsIntersect3 || IsIntersect4;
}

bool DoseCircleAnnualAndLineAnnulaIntersect(BPointCoordinate aCircleCenter, double aRadius, BPointCoordinate APoint, BPointCoordinate BPoint, double AnnualWidth)
{
	double radiusMin = aRadius - 0.5 * AnnualWidth;
	double radiusMax = aRadius + 0.5 * AnnualWidth;

	double A = APoint.Y - BPoint.Y;
	double B = BPoint.X - APoint.X;
	double C = APoint.X * BPoint.Y - BPoint.X * APoint.Y;

	double CMin = C + 0.5 * AnnualWidth / sqrt(A * A + B * B);
	double CMax = C - 0.5 * AnnualWidth / sqrt(A * A + B * B);

	double distanceBetweenCircleAndLine1 = fabs(A * aCircleCenter.X + B * aCircleCenter.Y + CMin) / sqrt(A * A + B * B);
	double distanceBetweenCircleAndLine2 = fabs(A * aCircleCenter.X + B * aCircleCenter.Y + CMax) / sqrt(A * A + B * B);

	if (distanceBetweenCircleAndLine1 < radiusMax || distanceBetweenCircleAndLine2 < radiusMax)
	{
		return true;
	}
	else
	{
		return false;
	}
}