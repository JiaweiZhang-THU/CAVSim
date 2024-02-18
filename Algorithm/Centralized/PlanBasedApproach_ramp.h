#ifndef _PLAN_BASED_APPROACH_RAMP
#define _PLAN_BASED_APPROACH_RAMP

#include<ctime>

#include"../Distributed/TrajectoryPlanner.h"
#include"../Distributed/TrajectoryPlanner.h"
#include"../../TestingRelated/ScheduleTree_ramp.h"
#include"TimingRecord.h"
#include"PlanRecord.h"
#include"../../Object/Road/MergingBlock.h"

class PlanBasedApproach_ramp {
public:
	PlanBasedApproach_ramp();
	PlanBasedApproach_ramp(int aRampMainRoadLanesNum, BPointCoordinate aConflictBoundaryPointOnMainRoad, BPointCoordinate aConflictBoundaryPointOnRamp, double aSimulatorTimeCount);
	~PlanBasedApproach_ramp();

	double ConsiderRadius;
	double LockRadius; 
	list<Vehicle> DrivingInVehiclesListArray[2];
	int RampMainRoadLanesNum;
	BPointCoordinate ConflictBoundaryPointOnMainRoad; 
	BPointCoordinate ConflictBoundaryPointOnRamp; 

	double SimulatorTimeCount;
	double SimulatorTimeDelta;

	list<Vehicle> DrvingPlan_SimuVehOrderList;
	TrajectoryPlanner* Planner;
	list<list<list<int>>> AllSeqList;
	list<list<int>> BestSeqList;
	list<int> AppliedPassingOrder;

	map<int, Vehicle> SimuVehicleDict;
	list<int> SimuIdList;

	ScheduleTree_ramp* _scheduleTree_ramp;

	list<Vehicle> LastLockingList;
	list<Vehicle> ToDeleteList;

	map<int, DrivingPlan> DrivingPlanDict;
	string AlgorithmInformation;
	list<TimingRecord> RecordList;

	TimingRecord* CurrRecord;

	list<PlanRecord> PlanRecordsList;

	double ConflictSubzonesLastPassedVehicleArrivalTime[20][20]; 
	double Distance2RampConflictCenter(Vehicle aVehicle);
	void UpdatePlanningResults(list<Vehicle>& Simulator_Vehicles, map<int, DrivingPlan>& Simulator_DrivingPlan);
	void UpdateRampArrivalTime(list<Vehicle>& Simulator_Vehicles);

	void Init();

	bool Run(map<int, DrivingPlan>& aExistVehicleDrivingPlan, bool& HasNewLockedVehicle);

	bool UpdateVehicleRegistrationInfo(map<int, DrivingPlan>& aExistVehicleDrivingPlan, bool& aHasNewLockedVehicle);

	void GetVehicleLaneIndex(int id, int& firstIndex, int& secondIndex);

	void GenerateNewPlan();

	void GenerateSchedule();

	void GenerateBestPlan();
};

#endif // !_PLAN_BASED_APPROACH_RAMP

