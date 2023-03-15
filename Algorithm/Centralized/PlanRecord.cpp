#include "PlanRecord.h"

PlanRecord::PlanRecord()
{
}

PlanRecord::~PlanRecord()
{
}

int PlanRecord::VehicleListFindIndex(list<Vehicle> aVehicleList, int aNum)
{
	if (aVehicleList.size() <= 0)
	{
		return -1;
	}
	list<Vehicle>::iterator VehicleIter = aVehicleList.begin();
	int i = 0;
	for (; i < aVehicleList.size(); i++, VehicleIter++)
	{
		if (VehicleIter->Id == aNum)
		{
			return i;
		}
	}
	return -1;

}

int PlanRecord::IntListIndex2Value(list<int> aIntList, int aIndex)
{
	list<int>::iterator IntIter = aIntList.begin();
	int i = 0;
	for (; i < aIntList.size(); i++, IntIter++)
	{
		if (i == aIndex)
		{
			return *IntIter;
		}
	}
	return NULL;
}

void PlanRecord::Generate(list<Vehicle> ConsiderVehicleList, list<list<int>> BestSeqList)
{
	PlanRecord::CarVectorList = *new list<CarVector>();

	ConsiderVehicleList.sort(sort_list);

	for (list<Vehicle>::iterator localVehicleIter = ConsiderVehicleList.begin(); localVehicleIter != ConsiderVehicleList.end(); localVehicleIter++)
	{
		CarVector localCarVector = *new CarVector();
		localCarVector.GenerateFromVehicle(*localVehicleIter);
		PlanRecord::CarVectorList.push_back(localCarVector);
	}

	list<int> WholeList;
	for (list<list<int>>::iterator IntIterIter = BestSeqList.begin(); IntIterIter != BestSeqList.end(); IntIterIter++)
	{
		for (list<int>::iterator IntIter = IntIterIter->begin(); IntIter != IntIterIter->end(); IntIter++)
		{
			WholeList.push_back(*IntIter);
		}
	}

	PlanRecord::Order = 0;
	int N = int(WholeList.size());
	list<int>::iterator IntIter = WholeList.begin();
	for (int i = 0; i < N - 1; i++,IntIter++)
	{
		int number = *IntIter;
		int index = PlanRecord::VehicleListFindIndex(ConsiderVehicleList, number);

		int shouldCutDown = 0;
		for (int k = 0; k < i; k++)
		{
			if (PlanRecord::VehicleListFindIndex(ConsiderVehicleList, PlanRecord::IntListIndex2Value(WholeList, k) < PlanRecord::VehicleListFindIndex(ConsiderVehicleList, PlanRecord::IntListIndex2Value(WholeList, i))))
			{
				shouldCutDown++;
			}
		}

		PlanRecord::Order = (PlanRecord::Order + index - shouldCutDown) * (N - 1 - i);
	}

	PlanRecord::MagicNum = 1;
	int cutValue = 1;
	list<int> CountList;
	for (list<list<int>>::iterator IntIterIter = BestSeqList.begin(); IntIterIter != BestSeqList.end(); IntIterIter++)
	{
		CountList.push_front(int(IntIterIter->size()));
	}
	
	for (list<int>::iterator IntIter = CountList.begin(); IntIter != CountList.end(); IntIter++)
	{
		cutValue = cutValue << *IntIter;
		PlanRecord::MagicNum += cutValue;
	}
}

int PlanRecord::GetCarCount()
{
	return int(PlanRecord::CarVectorList.size());
}

int PlanRecord::GetVectorCount()
{
	return 4 * int(PlanRecord::CarVectorList.size()) + 1;
}

string PlanRecord::ToString()
{
	list<CarVector>::iterator localCVIter;
	string localSb;
	for (localCVIter = PlanRecord::CarVectorList.begin(); localCVIter != PlanRecord::CarVectorList.end(); localCVIter++)
	{
		localSb += localCVIter->ToString();
	}

	localSb += to_string(PlanRecord::Order);

	return localSb;
}

bool PlanRecord::sort_list(Vehicle& aVehicle_1, Vehicle& aVehicle_2)
{
	if (aVehicle_1.FromNodeId == aVehicle_2.FromNodeId)
	{
		return aVehicle_1.LeftLaneDistance < aVehicle_2.LeftLaneDistance;
	}
	else
	{
		return aVehicle_1.FromNodeId < aVehicle_2.FromNodeId;
	}
}
