#pragma once
#ifndef _PASSING_STRATEGY_EVALUATION_
#define _PASSING_STRATEGY_EVALUATION_

#include <vector>
#include <map>
#include "../../Object/Vehicle/Vehicle.h"
using namespace std;

class IntersectionStrategyEvaluation
{
public:
	IntersectionStrategyEvaluation();
	~IntersectionStrategyEvaluation();

protected:
	int VehicleId2RecordId[5000];

	int NumVehiclesAdded;
	vector<double> RealTimeEnteringControlArea;	

	vector<double> RealTimeEnteringConflictArea;		

	vector<double> IdealTimeEnteringConflictArea;	

	vector<vector<double>*> SpeedRecordinControlArea;	

	double TotalDelay;

	double TotalEnergyConsumption;

	double TotalFuelConsumption;

public:
	double TimeStep;

	int NumVehiclesCalculated;

	const void GetEvalResult(double& AvgDelay, double& AvgEnergyConsumption, double& AvgFuelConsumption);

	void PrintEvalResult();
	
	const double AvgDelay();
	
	const double AvgEnergyConsumption();
	
	const double AvgFuelConsumption();

	string SavePath;


	void clear();

	void RunOneStep(Vehicle& aVehicle);

	void RunOneStepAtMerging(Vehicle& aVehicle);

	void RunWholeTrajectory(const int& Delay, const vector<double>& aSpeedRecord);

	const double CalcuEnergyConsumption(const vector<double>& aSpeedRecord);

	const double CalcuFuelConsumption(const vector<double>& aSpeedRecord);

protected:
	const int MinimumArrivalTime(const Vehicle& aVehicle);

	const double IntegralWithSimpson(const vector<double>& function);
};

#endif