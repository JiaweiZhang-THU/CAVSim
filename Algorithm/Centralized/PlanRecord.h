#ifndef _PLAN_RECORD_
#define _PLAN_RECORD_

#include "../../Object/Vehicle/Vehicle.h"

struct CarVector
{
public:
	int Id;
	int FromNodeId;
	int ToNodeId;
	int SpeedLevel;
	int DistLevel;

public:
	void GenerateFromVehicle(Vehicle aVehicle)
	{
		CarVector::Id = aVehicle.Id;
		CarVector::FromNodeId = aVehicle.FromNodeId;
		CarVector::ToNodeId = aVehicle.ToNodeId;

		CarVector::SpeedLevel = (int)aVehicle.Speed;
		CarVector::DistLevel = (int)aVehicle.LeftLaneDistance;
	}

	string ToString()
	{
		string localString = to_string(CarVector::FromNodeId) + " " + to_string(CarVector::ToNodeId) + " " + to_string(CarVector::SpeedLevel) + " " + to_string(CarVector::DistLevel) + " ";
		return localString;
	}
};

class PlanRecord
{
public:
	PlanRecord();
	~PlanRecord();

public:
	list<CarVector> CarVectorList;
	int Order;
	int MagicNum;

	void Generate(list<Vehicle> ConsiderVehicleList, list<list<int>> BestSeqList);

	int VehicleListFindIndex(list<Vehicle> aVehicleList, int aNum);
	int IntListIndex2Value(list<int> aIntList, int aIndex);

	int GetCarCount();

	int GetVectorCount();

	string ToString();

	static bool sort_list(Vehicle& aVehicle_1, Vehicle& aVehicle_2);

};

#endif // !_PLAN_RECORD_