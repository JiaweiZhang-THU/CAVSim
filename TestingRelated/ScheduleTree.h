#ifndef _SCHEDULE_TREE_
#define _SCHEDULE_TREE_

#include<stdio.h>
#include<iostream>
#include<list>
#include<ctime>
#include<map>
#include<vector>
#include<algorithm>
#include <functional>

#include"../Algorithm/Centralized/CollisionAvoidSequencePlanner.h"
#include "../../Object/Vehicle/Vehicle.h"
#include "../Algorithm/Centralized/TimingRecord.h"
#include "MCTS.h"
#include "ArrivalTime.h"

using namespace std;

static bool local_cmp(const pair<int, double>& a, const pair<int, double>& b) {
	return a.second < b.second;
}

class ScheduleTree
{
public:
	ScheduleTree();
	~ScheduleTree();

	ScheduleTree(list<Vehicle> aDrivingInVehiclesListArray[4 * 10], int aIntersectionOneWayLanesNum, BPointCoordinate aIntersectionCenter, map<int, Vehicle> aSimuVehicleDict, double aConflictSubzonesLastPassedVehicleArrivalTime[20][20], double aTimeNow);

	list<Vehicle> DrivingInVehiclesListArray[4*10]; 

	double LaneWidth;

	int IntersectionOneWayLanesNum; 

	BPointCoordinate IntersectionCenter;
	
	TimingRecord CurrRecord;

	map<int, Vehicle> SimuVehicleDict;

	double ToleranceThreshold;

	double ConflictSubzonesLastPassedVehicleArrivalTime[20][20];

	double TimeNow;

	double Distance2IntersectionCenter(Vehicle aVehicle);

	void AddToAllSeqList(list<list<list<int>>>& aAllSeqList, list<int>& aAppliedPassingOrder);

	void FindDistanceFIFOPassingOrder();

	void FindTimeFIFOPassingOrder();

	void FindMCTSPassingOrder();

	list<list<int>> AllPerList;

	list<list<list<int>>> AllSeqList;

	list<list<int>> OptimizeSeqList;

	list<int> AppliedPassingOrder;

	list<int> MCTSPassingOrder;

	list<int> DistanceFIFOPassingOrder;

	list<int> TimeFIFOPassingOrder;


	int* OnePossiblePermutationOrder;

	CollisionAvoidSequencePlanner* MyCollisionAvoidSequencePlanner;

	list<int> OriginSeqList[4*10];

	struct CorrPair
	{
		int A;
		int B;
		
		CorrPair(int a, int b)
		{
			CorrPair::A = a;
			CorrPair::B = b;
		}
	};

	void Init();

	void Load();

	void Run();

	void GenerateOriginSeqList();

	void InteriorEnumerationLoop(int leftCount, int sum);

	void GenerateAllSeqList();

	int GetAllPerListCount();

	double ArrivalTimeCalculByIterativeSolution(list<int> aPassingOrderList);

	void CalculatePassingIndex();

	bool isWiseToArrangeSo(Vehicle former, Vehicle latter);

private:

};

#endif // !_SCHEDULE_TREE_
