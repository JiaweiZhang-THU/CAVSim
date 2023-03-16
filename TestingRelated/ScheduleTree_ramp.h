#ifndef _SCHEDULE_TREE_RAMP
#define _SCHEDULE_TREE_RAMP

#include<stdio.h>
#include<iostream>
#include<list>
#include<ctime>
#include<map>
#include<vector>
#include<algorithm>
#include<functional>

#include"../Algorithm/Centralized/CollisionAvoidSequencePlanner.h"
#include"../Object/Vehicle/Vehicle.h"
#include"../Algorithm/Centralized/TimingRecord.h"
#include"ArrivalTime.h"

#include"DPMethod.h"
#include"GroupMethod.h"
#include"RuleMethod.h"

using namespace std;

class ScheduleTree_ramp {
public:
	ScheduleTree_ramp();
	~ScheduleTree_ramp();
	ScheduleTree_ramp(list<Vehicle> aDrivingInVehiclesListArray[2], BPointCoordinate aConflictBoundaryPointOnMainRoad, BPointCoordinate aConflictBoundaryPointOnRamp, map<int, Vehicle> aSimuVehicleDict, double aTimeNow);

	list<Vehicle> DrivingInVehiclesListArray[2];
	double LaneWidth;
	BPointCoordinate ConflictBoundaryPointOnMainRoad;
	BPointCoordinate ConflictBoundaryPointOnRamp;

	TimingRecord CurrRecord;

	map<int, Vehicle> SimuVehicleDict;

	double TimeNow;

	DPMethod* MyDPMethod;
	GroupMethod* MyGroupMethod;
	RuleMethod* MyRuleMethod;


	double Distance2RampConflictCenter(Vehicle aVehicle);

	void AddToAllSeqList(list<list<list<int>>>& aAllSeqList, list<int>& aAppliedPassingOrder);

	void FindDistanceFIFOPassingOrder();

	void FindTimeFIFOPassingOrder();

	void FindDPPassingOrder();

	void FindGroupingPassingOrder();

	void FindRulePassingOrder();

	list<list<int>> AllPerList;
	list<list<list<int>>> AllSeqList;
	list<list<int>> OptimizeSeqList;

	list<int> AppliedPassingOrder;

	list<int> DistanceFIFOPassingOrder;

	list<int> TimeFIFOPassingOrder;

	list<int> DPPassingOrder;

	list<int> GroupPassingOrder;

	list<int> RulePassingOrder;


	int* OnePossiblePremutationOrder;

	CollisionAvoidSequencePlanner* MyCollisionAvoidSequencePlanner;

	list<int> OriginSeqList[2];

	struct CorrPair {
		int A;
		int B;

		CorrPair(int a, int b) {
			CorrPair::A = a;
			CorrPair::B = b;
		}
	};

	void Init();

	void Load();

	void Run();

	void GenerateOriginSeqList();

	void EnumerateAllBasicOrder();

	void InteriorEnumerationLoop(int leftCount, int sum);

	void GenerateAllSeqList();

	int GetAllPerListCount();

	double ArrivalTimeCalcuByIterativeSolution(list<int> aPassingOrderList);

	void CalculatePassingIndex();

	bool isWiseToArrangeSo(Vehicle former, Vehicle latter);
private:

};
#endif // !_SCHEDULE_TREE_RAMP