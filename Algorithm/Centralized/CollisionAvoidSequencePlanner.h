#ifndef _COLLISION_AVOID_SEQUENCE_PLANNER_
#define _COLLISION_AVOID_SEQUENCE_PLANNER_
#include<iostream>
#include<list>
#include<stack>
#include<map>

#include "../../TestingRelated/CollisionDetection.h"

using namespace std;

class CollisionAvoidSequencePlanner
{
public:
	CollisionAvoidSequencePlanner();
	CollisionAvoidSequencePlanner(list<int> rawSeqList, int aIntersectionOneWayLanesNum, double aLaneWidth, BPointCoordinate aIntersectionCenter, map<int, Vehicle> aSimuVehicleDict, list<list<list<int>>> aPlanBasedApproach_AllSeqList);

	CollisionAvoidSequencePlanner(int aIntersectionOneWayLanesNum, double aLaneWidth, BPointCoordinate aIntersectionCenter, map<int, Vehicle> aSimuVehicleDict, list<list<list<int>>> aPlanBasedApproach_AllSeqList);


	~CollisionAvoidSequencePlanner();

	int IntersectionOneWayLanesNum;

	double LaneWidth;
	BPointCoordinate IntersectionCenter; 
	CollisionDetection MyCollisionDetection;

	map<int, Vehicle> SimuVehicleDict;

	list<list<list<int>>> PlanBasedApproach_AllSeqList;

	void AddToTheAllSeqListCite(list<list<list<int>>>& aAllSeqList);

public:
	class CollisionPair {
	public:
		int A;
		int B;
	};
public:
	
	list<CollisionPair> CollisionPairList;

	int CutThreshold;

	map<int,int> BasicSeqArray;
	int* _find;
	int* _next;
	int* _prev;
	int _offset;
	bool _cut[100];

	list<list<int>> FinalSeqList;
	stack<int> recordStack;
	list<int> BinarayPruneList;

	int sindex;


	void Init();

	void ResetSindex();

	void Load(list<int> rawSeqList, list<list<list<int>>> aPlanBasedApproach_AllSeqList);

	void Run();


	void FirstCheckNeighborCollision();


	void FindRestCollisionPairs();

	void PruneTreeAlgor();


	list<CollisionPair> PruneCollisionPairs(int index);

	int CountInitialCutNum();

	void InteriorAlgorLoop(int cutNum);

	void RemoveThoseCutTooMuch();


	void RemoveThoseIsAnotherPermutation();


	void UpdateNextAndPrevByAddCut(int index);


	void UpdateNextAndPrevByDelCut(int index);


	void GenerateFinalOrder();


	void AddToTheAllSeqList();

	int generateCutBinary();

	bool isRedundant();

	bool Debug_Is_Corrected();

	bool Debug_Do_Something();
private:

};


#endif // !_COLLISION_AVOID_SEQUENCE_PLANNER_

