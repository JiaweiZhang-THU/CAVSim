#include "ArrivalTime.h"
#include "ArricalTimeParameters.h"
ArrivalTime::ArrivalTime()
{
}

ArrivalTime::~ArrivalTime()
{
}

ArrivalTime::ArrivalTime(double aLargestArrivalTimeInConflictSubzones[20][20], int aLaneNum)
{
	for (int i = 0; i < 2 * aLaneNum; i++)
	{
		for (int j = 0; j < 2 * aLaneNum; j++)
		{
			ArrivalTime::LargestArrivalTimeInConflictSubzones[i][j] = aLargestArrivalTimeInConflictSubzones[i][j];
		}
	}
}

void ArrivalTime::CalculMinimumArrivalTimeToOtherSubconflcitzones(Vehicle& aVehicle)
{
	if (aVehicle.SteeringType == 0)
	{
		aVehicle.MinimumArrivalTimeToConflictSubzones[0] = aVehicle.MinimumArrivalTime;

		double DeltaTime = paraLaneWidth4 / aVehicle.MaxSteerSpeed;

		aVehicle.DeltaTimeBetweenTwoConflictSubzone = DeltaTime;

		for (int i = 1; i < aVehicle.TrajectoryCoverageConflictSubzonesArraySize; i++)
		{
			aVehicle.MinimumArrivalTimeToConflictSubzones[i] = aVehicle.MinimumArrivalTime + i * DeltaTime;
		}
	}
	else
	{
		aVehicle.MinimumArrivalTimeToConflictSubzones[0] = aVehicle.MinimumArrivalTime;

		double DeltaTime = (0.5 * acos(-1) * aVehicle.SteeringRadius / aVehicle.MaxSteerSpeed) / aVehicle.TrajectoryCoverageConflictSubzonesArraySize;

		aVehicle.DeltaTimeBetweenTwoConflictSubzone = DeltaTime;
		
		for (int i = 1; i < aVehicle.TrajectoryCoverageConflictSubzonesArraySize; i++)
		{
			aVehicle.MinimumArrivalTimeToConflictSubzones[i] = aVehicle.MinimumArrivalTime + i * DeltaTime;
		}
	}
}

double ArrivalTime::CalculMinimumArrivalTimeBangBangControl(Vehicle aVehicle, double aSimuTimeNow)
{
	double Di = aVehicle.LeftLaneDistance;
	double V1_i = sqrt(
		(2 * Di * aVehicle.MaxStraightAccel * aVehicle.MinStraightAccel + aVehicle.MinStraightAccel * aVehicle.Speed * aVehicle.Speed - aVehicle.MaxStraightAccel * aVehicle.MaxSteerSpeed * aVehicle.MaxSteerSpeed) 
		/ (aVehicle.MinStraightAccel - aVehicle.MaxStraightAccel)
	);

	double tf_i_min = 0;
	if (V1_i <= aVehicle.MaxStraightSpeed)
	{
		tf_i_min = aSimuTimeNow + (V1_i - aVehicle.Speed) / aVehicle.MaxStraightAccel + (aVehicle.MaxSteerSpeed - V1_i) / aVehicle.MinStraightAccel;
	}
	else
	{
		tf_i_min = aSimuTimeNow + 
			(aVehicle.MaxStraightSpeed - aVehicle.Speed) / aVehicle.MaxStraightAccel + 
			(aVehicle.MaxSteerSpeed - aVehicle.MaxStraightSpeed) / aVehicle.MinStraightAccel + 
			Di / aVehicle.MaxStraightSpeed - 
			(aVehicle.MaxStraightSpeed * aVehicle.MaxStraightSpeed - aVehicle.Speed * aVehicle.Speed) / (2 * aVehicle.MaxStraightAccel * aVehicle.MaxStraightSpeed) + 
			(aVehicle.MaxStraightSpeed * aVehicle.MaxStraightSpeed - aVehicle.MaxSteerSpeed * aVehicle.MaxSteerSpeed) / (2 * aVehicle.MinStraightAccel * aVehicle.MaxStraightSpeed);
	}

	return tf_i_min;
}

double ArrivalTime::CalculMinimumArrivalTimeAtOnRamp(Vehicle aVehicle, double aSimuTimeNow)
{
	double Di = aVehicle.LeftLaneDistance;
	double Vi = aVehicle.Speed;
	double a_max = aVehicle.MaxStraightAccel;
	double v_max = aVehicle.MaxStraightSpeed;
	double t_min;
	if ((v_max * v_max - Vi * Vi) / a_max > 2 * Di) {
		t_min = aSimuTimeNow + (sqrt(Vi * Vi + 2 * a_max * Di) - Vi) / a_max;
	}
	else {
		t_min = aSimuTimeNow + (v_max - Vi) / a_max + (Di - (v_max * v_max - Vi * Vi) / 2 / a_max) / v_max;
	}
	return t_min;
}


void ArrivalTime::AdjustTimeAssignAccordingToTheConstantVelocityInTheConflictZone(Vehicle& aVehicle)
{
	double* AssignedTime = new double[aVehicle.TrajectoryCoverageConflictSubzonesArraySize];
	for (int i = 0; i < aVehicle.TrajectoryCoverageConflictSubzonesArraySize; i++)
	{
		AssignedTime[i] = aVehicle.AssignedArrivalTimeToConflictSubzones[i] - i * aVehicle.DeltaTimeBetweenTwoConflictSubzone;
	}

	double MaxEnteringTime = AssignedTime[0];
	for (int i = 1; i < aVehicle.TrajectoryCoverageConflictSubzonesArraySize; i++)
	{
		if (AssignedTime[i] > MaxEnteringTime)
		{
			MaxEnteringTime = AssignedTime[i];
		}
	}

	aVehicle.AssignedTimeToArriveConfliceZone = MaxEnteringTime;

	if (aVehicle.SteeringType == 0)
	{
		aVehicle.AssignedArrivalTimeToConflictSubzones[0] = aVehicle.AssignedTimeToArriveConfliceZone;

		double DeltaTime = paraLaneWidth4 / aVehicle.MaxSteerSpeed;

		aVehicle.DeltaTimeBetweenTwoConflictSubzone = DeltaTime;

		for (int i = 1; i < aVehicle.TrajectoryCoverageConflictSubzonesArraySize; i++)
		{
			aVehicle.AssignedArrivalTimeToConflictSubzones[i] = aVehicle.AssignedTimeToArriveConfliceZone + i * DeltaTime;
		}
	}
	else
	{
		aVehicle.AssignedArrivalTimeToConflictSubzones[0] = aVehicle.AssignedTimeToArriveConfliceZone;

		double DeltaTime = (0.5 * acos(-1) * aVehicle.SteeringRadius / aVehicle.MaxSteerSpeed) / aVehicle.TrajectoryCoverageConflictSubzonesArraySize;

		aVehicle.DeltaTimeBetweenTwoConflictSubzone = DeltaTime;

		for (int i = 1; i < aVehicle.TrajectoryCoverageConflictSubzonesArraySize; i++)
		{
			aVehicle.AssignedArrivalTimeToConflictSubzones[i] = aVehicle.AssignedTimeToArriveConfliceZone + i * DeltaTime;
		}
	}
}

double ArrivalTime::PassingOrderToTrajectoryInterPretaton(list<int> aPassingOrder, map<int, Vehicle>& aSimuVehicleDict, double aSimuTimeNow)
{
	for (list<int>::iterator IIter = aPassingOrder.begin(); IIter != aPassingOrder.end(); IIter++)
	{
		Vehicle& localVehicle = aSimuVehicleDict[*IIter];

		localVehicle.MinimumArrivalTime = ArrivalTime::CalculMinimumArrivalTimeBangBangControl(localVehicle, aSimuTimeNow);

		ArrivalTime::CalculMinimumArrivalTimeToOtherSubconflcitzones(localVehicle);

		for (int z = 0; z < localVehicle.TrajectoryCoverageConflictSubzonesArraySize; z++)
		{
			int localConflictSubzone = localVehicle.TrajectoryCoverageConflictSubzonesArray[z] - 1;

			int Xindex = int((int)localConflictSubzone / (2 * paraLaneNum2));
			int Yindex = int((int)localConflictSubzone % (int)(2 * paraLaneNum2));

			localVehicle.AssignedArrivalTimeToConflictSubzones[z] = max(localVehicle.MinimumArrivalTimeToConflictSubzones[z], ArrivalTime::LargestArrivalTimeInConflictSubzones[Xindex][Yindex] + paraDeltaTimeBetweenTwoVehicle);
			
		}

		ArrivalTime::AdjustTimeAssignAccordingToTheConstantVelocityInTheConflictZone(localVehicle);

		for (int z = 0; z < localVehicle.TrajectoryCoverageConflictSubzonesArraySize; z++)
		{
			int localConflictSubzone = localVehicle.TrajectoryCoverageConflictSubzonesArray[z] - 1;

			int Xindex = int((int)localConflictSubzone / (2 * paraLaneNum2));
			int Yindex = int((int)localConflictSubzone % (int)(2 * paraLaneNum2));

			ArrivalTime::LargestArrivalTimeInConflictSubzones[Xindex][Yindex] = localVehicle.AssignedArrivalTimeToConflictSubzones[z];
		}
		
	}

	double JTotalDelay = 0;
	for (list<int>::iterator IIter = aPassingOrder.begin(); IIter != aPassingOrder.end(); IIter++)
	{
		Vehicle localVehicle = aSimuVehicleDict[*IIter];

		JTotalDelay += localVehicle.AssignedTimeToArriveConfliceZone;
	}
	return JTotalDelay;
}