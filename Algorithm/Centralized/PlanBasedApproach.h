#ifndef _PLAN_BASED_APPROACH_
#define _PALN_BASED_APPROACH_

#include <ctime>

#include"../Distributed/TrajectoryPlanner.h"
#include "../../TestingRelated/ScheduleTree.h"
#include "TimingRecord.h"
#include "PlanRecord.h"
#include "../../Object/Road/Intersection.h"

class PlanBasedApproach
{
public:
	PlanBasedApproach();
	PlanBasedApproach(int aIntersectionOneWayLanesNum, BPointCoordinate aIntersectionCenter, double aSimulatorTimeCount);
	~PlanBasedApproach();

public:

	bool ConsiderAllCut;

	double OuterRadius;

	list<Vehicle> DrivingInVehiclesListArray[4*10];

	int IntersectionOneWayLanesNum;

	BPointCoordinate IntersectionCenter; 

	double SimulatorTimeCount;

	double SImulatorTimeDelta;

	list<Vehicle> DrivingPlan_SimuVehOrderList;

	void Load(int aIntersectionOneWayLanesNum, BPointCoordinate aIntersectionCenter, double aSimulatorTimeCount);

	void Load(Intersection aIntersection);

	int ConflictSubzonesLastPassedVehicleID[20][20];

	double ConflictSubzonesLastPassedVehicleArrivalTime[20][20];

	double Distance2IntersectionCenter(Vehicle aVehicle);

	void UpdatePlanningResults(list<Vehicle>& Simulator_Vehicles, map<int, DrivingPlan>& Simulator_DrivingPLan);

	void UpdateIntersectionArrivalTime(list<Vehicle>& Simulator_Vehicles, Intersection& aIntersection);

	double LockRaduius;

	double ConsiderRadius;

	TrajectoryPlanner* Planner;

	list<list<list<int>>> AllSeqList;

	list<list<int>> BestSeqList;

	list<int> AppliedPassingOrder;

	map<int, Vehicle> SimuVehicleDict;

	list<int> SimuIdList;

	ScheduleTree* _scheduleTree;

	list<Vehicle> LastLockingList;

	list<Vehicle> ToDeleteList;

	map<int, DrivingPlan> DrivingPlanDict;
	
	string AlgorInformation;

	list<TimingRecord> RecordList;

	TimingRecord* CurrRecord;

	list<PlanRecord> PlanRecordsList;

	void Init();

	bool Run(map<int, DrivingPlan>& aExistVehicleDrivingPlan, bool& HasNewLockedVehicle);

	bool UpdateVehicleRegistrationInfo(map<int, DrivingPlan>& aExistVehicleDrivingPlan, bool& aHasNewLockedVehicle);

	void GetVehicleLaneIndex(int id, int& firstIndex, int& secondIndex);

	void GenerateNewPlan();

	void GenerateSchedule();

	void GenerateBestPlan();

	void FollowAllocatedPlan();

private:

};

#endif // !_PLAN_BASED_APPROACH_
