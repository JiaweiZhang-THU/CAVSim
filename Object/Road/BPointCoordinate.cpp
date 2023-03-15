#include<cmath>
#include "BPointCoordinate.h"

BPointCoordinate::BPointCoordinate(void)
{
	BPointCoordinate::X = 0;
	BPointCoordinate::Y = 0;
	BPointCoordinate::ScaleFactorForDisplay = 2;
}

BPointCoordinate::BPointCoordinate(double _X, double _Y)
{
	BPointCoordinate::X = _X;
	BPointCoordinate::Y = _Y;
	BPointCoordinate::ScaleFactorForDisplay = 2;
}

BPointCoordinate::BPointCoordinate(double _X, double _Y, double _ScaleFactorForDisplay)
{
	BPointCoordinate::X = _X;
	BPointCoordinate::Y = _Y;
	BPointCoordinate::ScaleFactorForDisplay = _ScaleFactorForDisplay;
}

double BPointCoordinate::GetDistance(BPointCoordinate point1, BPointCoordinate point2)
{
	double X_distance = fabs(point1.X - point2.X);
	double Y_distance = fabs(point1.Y - point2.Y);
	return sqrt(X_distance * X_distance + Y_distance * Y_distance);
}

double GetDistance(double point1_X, double point1_Y, double point2_X, double point2_Y)
{
	double X_distance = fabs(point1_X - point2_X);
	double Y_distance = fabs(point1_Y - point2_Y);
	return sqrt(X_distance * X_distance + Y_distance * Y_distance);
}

BPointCoordinate BPointCoordinate::DeepCopy(void)
{
	BPointCoordinate NewCopiedPoint(BPointCoordinate::X, BPointCoordinate::Y, BPointCoordinate::ScaleFactorForDisplay);
	return NewCopiedPoint;
}

double BPointCoordinate::GetPixelXFromMeterX(double meterX)
{
	return meterX * BPointCoordinate::ScaleFactorForDisplay;
}

double BPointCoordinate::GetPixelYFromMeterY(double meterY)
{
	return meterY * BPointCoordinate::ScaleFactorForDisplay;
}

double BPointCoordinate::GetPixelFromMeter(double meter)
{
	return meter * BPointCoordinate::ScaleFactorForDisplay;
}

void BPointCoordinate::SetScaleFactor(double ScaleFactor)
{
	BPointCoordinate::ScaleFactorForDisplay = ScaleFactor;
}

void BPointCoordinate::SetXAndY(double aX, double aY)
{
	BPointCoordinate::X = aX;
	BPointCoordinate::Y = aY;
}