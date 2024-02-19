#include "IntersectionStrategyEvaluation.h"
#include "IntersectionStrategyEvaluationParameters.h"
#include <iostream>
#include <fstream>

IntersectionStrategyEvaluation::IntersectionStrategyEvaluation()
{
	IntersectionStrategyEvaluation::TimeStep = paraEvalTimeStep;
	IntersectionStrategyEvaluation::NumVehiclesAdded = 0;
	IntersectionStrategyEvaluation::NumVehiclesCalculated = 0;
	IntersectionStrategyEvaluation::TotalDelay = 0;
	IntersectionStrategyEvaluation::TotalEnergyConsumption = 0;
	IntersectionStrategyEvaluation::TotalFuelConsumption = 0;
	memset(IntersectionStrategyEvaluation::VehicleId2RecordId, -1, sizeof(IntersectionStrategyEvaluation::VehicleId2RecordId));
}


IntersectionStrategyEvaluation::~IntersectionStrategyEvaluation()
{
	for (vector<vector<double>*>::iterator RecordIter = IntersectionStrategyEvaluation::SpeedRecordinControlArea.begin(); RecordIter != SpeedRecordinControlArea.end(); RecordIter++)
	{
		(*RecordIter)->clear();
		delete *RecordIter;
	}
}

const int IntersectionStrategyEvaluation::MinimumArrivalTime(const Vehicle& aVehicle)
{
	double Max_Speed = aVehicle.MaxStraightSpeed;
	double Max_Accel = aVehicle.MaxStraightAccel;
	double Dist = aVehicle.LeftLaneDistance;
	double Speed = aVehicle.Speed;
	double t;
	if (Max_Speed * Max_Speed - Speed * Speed >= 2 * Max_Accel * Dist)
	{
		t = 1 / Max_Accel * (-Speed + sqrt(Speed * Speed + 2 * Max_Accel * Dist));
	}
	else
	{
		t = (Max_Speed - Speed) / Max_Accel + (Dist - (Max_Speed * Max_Speed - Speed * Speed) / (2 * Max_Accel)) / Max_Speed;
	}
	return round(t * 10);
}

void IntersectionStrategyEvaluation::RunOneStep(Vehicle& aVehicle)
{
	int RecordId = IntersectionStrategyEvaluation::VehicleId2RecordId[aVehicle.Id];

	if (RecordId < 0)
	{
		if (aVehicle.TimeOfEnteringIntersectionCircleControlZone >= 0)
		{

			return;
		}
		else
		{
			IntersectionStrategyEvaluation::VehicleId2RecordId[aVehicle.Id] = IntersectionStrategyEvaluation::NumVehiclesAdded;
			RecordId = NumVehiclesAdded;
			IntersectionStrategyEvaluation::NumVehiclesAdded += 1;
			IntersectionStrategyEvaluation::RealTimeEnteringControlArea.push_back(-1);
			IntersectionStrategyEvaluation::RealTimeEnteringConflictArea.push_back(-1);
			IntersectionStrategyEvaluation::IdealTimeEnteringConflictArea.push_back(-1);
			IntersectionStrategyEvaluation::SpeedRecordinControlArea.push_back(new vector<double>);
		}
	}

	if (IntersectionStrategyEvaluation::RealTimeEnteringConflictArea[RecordId] >= 0)
	{
		return;
	}

	if (IntersectionStrategyEvaluation::RealTimeEnteringControlArea[RecordId] < 0)
	{
		if (aVehicle.TimeOfEnteringIntersectionCircleControlZone >= 0)
		{
			IntersectionStrategyEvaluation::RealTimeEnteringControlArea[RecordId] = aVehicle.TimeOfEnteringIntersectionCircleControlZone;
			IntersectionStrategyEvaluation::IdealTimeEnteringConflictArea[RecordId] = IntersectionStrategyEvaluation::MinimumArrivalTime(aVehicle) + aVehicle.TimeOfEnteringIntersectionCircleControlZone;
			IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]->push_back(aVehicle.Speed);
		}
		return;
	}

	if (aVehicle.TimeOfEnteringIntersectionConflictZone >= 0)
	{
		IntersectionStrategyEvaluation::RealTimeEnteringConflictArea[RecordId] = aVehicle.TimeOfEnteringIntersectionConflictZone;
		IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]->push_back(aVehicle.Speed);
		IntersectionStrategyEvaluation::NumVehiclesCalculated += 1;


		aVehicle.DelayOfPassingIntersection = IntersectionStrategyEvaluation::RealTimeEnteringConflictArea[RecordId] - IntersectionStrategyEvaluation::IdealTimeEnteringConflictArea[RecordId];
		aVehicle.EnergyConsumpingOfPassingIntersection = IntersectionStrategyEvaluation::CalcuEnergyConsumption(*IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]);
		aVehicle.FuelConsumptionOfPassingIntersection = IntersectionStrategyEvaluation::CalcuFuelConsumption(*IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]);

		IntersectionStrategyEvaluation::TotalDelay += aVehicle.DelayOfPassingIntersection;
		IntersectionStrategyEvaluation::TotalEnergyConsumption += aVehicle.EnergyConsumpingOfPassingIntersection;
		IntersectionStrategyEvaluation::TotalFuelConsumption += aVehicle.FuelConsumptionOfPassingIntersection;

		return;
	}

	IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]->push_back(aVehicle.Speed);
}


void IntersectionStrategyEvaluation::RunOneStepAtMerging(Vehicle& aVehicle)
{
	int RecordId = IntersectionStrategyEvaluation::VehicleId2RecordId[aVehicle.Id];

	if (RecordId < 0)
	{
		if (aVehicle.TimeOfEnteringIntersectionCircleControlZone >= 0)
		{
			return;
		}
		else
		{
			IntersectionStrategyEvaluation::VehicleId2RecordId[aVehicle.Id] = IntersectionStrategyEvaluation::NumVehiclesAdded;
			RecordId = NumVehiclesAdded;
			IntersectionStrategyEvaluation::NumVehiclesAdded += 1;
			IntersectionStrategyEvaluation::RealTimeEnteringControlArea.push_back(-1);
			IntersectionStrategyEvaluation::RealTimeEnteringConflictArea.push_back(-1);
			IntersectionStrategyEvaluation::IdealTimeEnteringConflictArea.push_back(-1);
			IntersectionStrategyEvaluation::SpeedRecordinControlArea.push_back(new vector<double>);
		}
	}
	if (IntersectionStrategyEvaluation::RealTimeEnteringConflictArea[RecordId] >= 0)
	{
		return;
	}
	if (IntersectionStrategyEvaluation::RealTimeEnteringControlArea[RecordId] < 0)
	{
		if (aVehicle.TimeOfEnteringIntersectionCircleControlZone >= 0)
		{
			IntersectionStrategyEvaluation::RealTimeEnteringControlArea[RecordId] = aVehicle.TimeOfEnteringIntersectionCircleControlZone;
			IntersectionStrategyEvaluation::IdealTimeEnteringConflictArea[RecordId] = IntersectionStrategyEvaluation::MinimumArrivalTime(aVehicle) + aVehicle.TimeOfEnteringIntersectionCircleControlZone;
			IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]->push_back(aVehicle.Speed);
		}
		return;
	}
	if (aVehicle.TimeOfEnteringIntersectionConflictZone >= 0)
	{
		IntersectionStrategyEvaluation::RealTimeEnteringConflictArea[RecordId] = aVehicle.TimeOfEnteringIntersectionConflictZone;
		IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]->push_back(aVehicle.Speed);
		IntersectionStrategyEvaluation::NumVehiclesCalculated += 1;


		aVehicle.DelayOfPassingIntersection = IntersectionStrategyEvaluation::RealTimeEnteringConflictArea[RecordId] - IntersectionStrategyEvaluation::IdealTimeEnteringConflictArea[RecordId];
		aVehicle.EnergyConsumpingOfPassingIntersection = IntersectionStrategyEvaluation::CalcuEnergyConsumption(*IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]);
		aVehicle.FuelConsumptionOfPassingIntersection = IntersectionStrategyEvaluation::CalcuFuelConsumption(*IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]);

		IntersectionStrategyEvaluation::TotalDelay += aVehicle.DelayOfPassingIntersection;
		IntersectionStrategyEvaluation::TotalEnergyConsumption += aVehicle.EnergyConsumpingOfPassingIntersection;
		IntersectionStrategyEvaluation::TotalFuelConsumption += aVehicle.FuelConsumptionOfPassingIntersection;

		return;
	}
	IntersectionStrategyEvaluation::SpeedRecordinControlArea[RecordId]->push_back(aVehicle.Speed);
}

void IntersectionStrategyEvaluation::RunWholeTrajectory(const int& Delay, const vector<double>& aSpeedRecord)
{
	IntersectionStrategyEvaluation::NumVehiclesCalculated += 1;
	IntersectionStrategyEvaluation::TotalDelay += Delay;
	IntersectionStrategyEvaluation::TotalEnergyConsumption += IntersectionStrategyEvaluation::CalcuEnergyConsumption(aSpeedRecord);
	IntersectionStrategyEvaluation::TotalFuelConsumption += IntersectionStrategyEvaluation::CalcuFuelConsumption(aSpeedRecord);
}


const double IntersectionStrategyEvaluation::CalcuEnergyConsumption(const vector<double>& aSpeedRecord)
{
	if (aSpeedRecord.size() < 3)
	{
		return 0;
	}
	cout << endl;
	vector<double> AccelRecord;
	vector<double>::const_iterator RecordIter0 = aSpeedRecord.begin();
	vector<double>::const_iterator RecordIter1 = aSpeedRecord.begin();
	RecordIter1++;
	int i = -1;
	while (RecordIter1 != aSpeedRecord.end())
	{
		AccelRecord.push_back(pow((*RecordIter1 - *RecordIter0) / IntersectionStrategyEvaluation::TimeStep, 2));
		RecordIter0++;
		RecordIter1++;
	}
	if (AccelRecord.size() < 3)
	{
		return 0;
	}
	double EnergyConsumption = IntersectionStrategyEvaluation::IntegralWithSimpson(AccelRecord);
	return EnergyConsumption;
}

const double IntersectionStrategyEvaluation::CalcuFuelConsumption(const vector<double>& aSpeedRecord)
{
	if (aSpeedRecord.size() < 3)
	{
		return 0;
	}
	vector<double> FuelRecord;
	vector<double>::const_iterator RecordIter0 = aSpeedRecord.begin();
	vector<double>::const_iterator RecordIter1 = aSpeedRecord.begin();
	RecordIter1++;
	double f_cruise, f_accel;
	double LocalSpeed, LocalAccel;
	while (RecordIter1 != aSpeedRecord.end())
	{
		LocalSpeed = *RecordIter0;
		LocalAccel = (*RecordIter1 - *RecordIter0) / IntersectionStrategyEvaluation::TimeStep;
		f_cruise = paraFuel_b0 + paraFuel_b1 * LocalSpeed + paraFuel_b2 * LocalSpeed * LocalSpeed + paraFuel_b3 * LocalSpeed * LocalSpeed * LocalSpeed;
		if (LocalAccel > 0)
		{
			f_accel = LocalAccel * (paraFuel_c0 + paraFuel_c1 * LocalSpeed + paraFuel_c2 * LocalSpeed * LocalSpeed);
		}
		else
		{
			f_accel = 0;
		}
		FuelRecord.push_back(f_cruise + f_accel);
		RecordIter0++;
		RecordIter1++;
	}
	double FuelConsumption = IntersectionStrategyEvaluation::IntegralWithSimpson(FuelRecord);
	return FuelConsumption;
}


const double IntersectionStrategyEvaluation::IntegralWithSimpson(const vector<double>& function)
{
	int N = (int(function.size()) - 1) / 2;
	bool IsEvenPoints = (function.size() - 1) % 2;
	vector<double>::const_iterator It0 = function.begin();
	vector<double>::const_iterator It1 = function.begin();
	double IntVal = 0;

	for (int i = 0; i < N; i++)
	{
		double local = function[2 * i] + 4 * function[2 * i + 1] + function[2 * i + 2];
		IntVal += IntersectionStrategyEvaluation::TimeStep / 3 * (function[2 * i] + 4 * function[2 * i + 1] + function[2 * i + 2]);
	}

	if (IsEvenPoints)
	{
		IntVal += IntersectionStrategyEvaluation::TimeStep / 2 * (function[function.size() - 1] + function[function.size() - 2]);
	}
	return IntVal;
}

const void IntersectionStrategyEvaluation::GetEvalResult(double& AvgDelay, double& AvgEnergyConsumption, double& AvgFuelConsumption)
{
	if (IntersectionStrategyEvaluation::NumVehiclesCalculated > 0)
	{
		AvgDelay = IntersectionStrategyEvaluation::TotalDelay / IntersectionStrategyEvaluation::NumVehiclesCalculated * IntersectionStrategyEvaluation::TimeStep;
		AvgDelay = IntersectionStrategyEvaluation::TotalEnergyConsumption / IntersectionStrategyEvaluation::NumVehiclesCalculated;
		AvgDelay = IntersectionStrategyEvaluation::TotalFuelConsumption / IntersectionStrategyEvaluation::NumVehiclesCalculated;
	}
	else
	{
		AvgDelay = -1;
		AvgEnergyConsumption = -1;
		AvgFuelConsumption = -1;
	}
}

const double IntersectionStrategyEvaluation::AvgDelay()
{
	if (IntersectionStrategyEvaluation::NumVehiclesCalculated > 0)
	{
		return IntersectionStrategyEvaluation::TotalDelay / IntersectionStrategyEvaluation::NumVehiclesCalculated * IntersectionStrategyEvaluation::TimeStep;
	}
	return -1;
}

const double IntersectionStrategyEvaluation::AvgEnergyConsumption()
{
	if (IntersectionStrategyEvaluation::NumVehiclesCalculated > 0)
	{
		return IntersectionStrategyEvaluation::TotalEnergyConsumption / IntersectionStrategyEvaluation::NumVehiclesCalculated;
	}
	return -1;
}

const double IntersectionStrategyEvaluation::AvgFuelConsumption()
{
	if (IntersectionStrategyEvaluation::NumVehiclesCalculated > 0)
	{
		return IntersectionStrategyEvaluation::TotalFuelConsumption / IntersectionStrategyEvaluation::NumVehiclesCalculated;
	}
	return -1;
}


void IntersectionStrategyEvaluation::clear()
{
	for (vector<vector<double>*>::iterator RecordIter = IntersectionStrategyEvaluation::SpeedRecordinControlArea.begin(); RecordIter != SpeedRecordinControlArea.end(); RecordIter++)
	{
		(*RecordIter)->clear();
		delete* RecordIter;
	}
	IntersectionStrategyEvaluation::RealTimeEnteringControlArea.clear();
	IntersectionStrategyEvaluation::RealTimeEnteringConflictArea.clear();
	IntersectionStrategyEvaluation::IdealTimeEnteringConflictArea.clear();
	IntersectionStrategyEvaluation::SpeedRecordinControlArea.clear();
	IntersectionStrategyEvaluation::NumVehiclesAdded = 0;
	IntersectionStrategyEvaluation::NumVehiclesCalculated = 0;
	IntersectionStrategyEvaluation::TotalDelay = 0;
	IntersectionStrategyEvaluation::TotalEnergyConsumption = 0;
	IntersectionStrategyEvaluation::TotalFuelConsumption = 0;
	memset(IntersectionStrategyEvaluation::VehicleId2RecordId, -1, sizeof(IntersectionStrategyEvaluation::VehicleId2RecordId));
}
