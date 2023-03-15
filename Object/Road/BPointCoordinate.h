#ifndef _BPointCoordinate_
#define _BPointCoordinate_

#pragma once
class BPointCoordinate {
public: 
	double X;
	double Y;

	double ScaleFactorForDisplay;

	BPointCoordinate(void);

	BPointCoordinate(double _X, double _Y);

	BPointCoordinate(double _X, double _Y, double _ScaleFactorForDisplay);

	double GetDistance(BPointCoordinate point1, BPointCoordinate point2);

	double GetDistance(double point1_X, double point1_Y, double point2_X, double point2_Y);

	BPointCoordinate DeepCopy(void);

	double GetPixelXFromMeterX(double meterX);
	double GetPixelYFromMeterY(double meterY);
	double GetPixelFromMeter(double meter);

	void SetXAndY(double aX, double aY);

	void SetScaleFactor(double ScaleFactor);

};

#endif // !_BPointCoordinate_