#ifndef _TRAJECTORY_PLANNER_
#define _TRAJECTORY_PLANNER_

#include<map>
#include "../Centralized/DrivingPlan.h"
#include "../../Object/Vehicle/Vehicle.h"
#include "../../TestingRelated/CollisionDetection.h"
#include "FollowingBehaviors.h"
#include "../../TestingRelated/ArrivalTime.h"
#include "EnergyOptimalControl2.h"

class TrajectoryPlanner
{
public:
	TrajectoryPlanner();
	TrajectoryPlanner(
		double aSimulator_TimeCount, 
		double aLaneNum, 
		double aOuterRadius, 
		list<Vehicle> aDrivingInVehiclesListArray[4 * 10], 
		list<Vehicle> aLastLockingList, 
		map<int, Vehicle> aSimuVehicleDict, 
		map<int, DrivingPlan> aPlanBasedApproach_DrivingPlanDict,
		double aConflictSubzonesLastPassedVehicleArrivalTime[20][20],
		list<int> aPassingOrderList
	);
	~TrajectoryPlanner();

public:

	list<Vehicle> DrivingInVehiclesListArray[4 * 10];

	list<Vehicle> LastLockingList;
	map<int, Vehicle> SimuVehicleDict;
	CollisionDetection MyCollisionDetection;

	double DeltaTime;
	double Simulator_TimeCount;

	double LaneNum;
	double LaneWidth;

	double OuterRadius;

	double MaintainingDistance;

	map<int, DrivingPlan> PlanBasedApproach_DrivingPlanDict;

	FollowingBehaviors MyFollowingBehaviors;

	double ConflictSubzonesLastPassedVehicleArrivalTime[20][20];


	EnergyOptimalControl2 MyOptimalController;

	void SceneNormalization(list<Vehicle>& aVehiclesList, Vehicle& aVehicle);
	list<Vehicle> LeadingVehicleList(list<Vehicle> aVehiclesList, Vehicle aVehicle);
	Vehicle LeadVehicle(list<Vehicle> aVehiclesList);

	void UploadPlanningResult(map<int, DrivingPlan>& aPlanBasedApproach_DrivingPlanDict, map<int, Vehicle>& aSimuVehicleDict);

	double ArrivalTimeCalculByIterativeSolution();

	void AnalyticalTrajectoryPLanning(DrivingPlan& plan, Vehicle simuVehicle);

	void TrajectoryPlanningInMergeZone(Vehicle simuVehicle, double simuInterDistance, BPointCoordinate simuLocation, double simuAngle, double simuOmega, DrivingPlan& plan);

	double Epsilon = 0.0000001;

	list<Vehicle> PlanningVehicleList;
	map<int, DrivingPlan> _drivingPlanDict;

	list<int> PassingOrderList;

	void Init();

	void Load(map<int, DrivingPlan> aDrivingPlanDict, list<list<int>> aSeqList, map<int,Vehicle> aSimuVehicleDict);

	void BindingPlanWithVehicle(int vehicleId);

	void Run(double& calDelay, double& estDelay, bool Concised,double& aTotalDelay);

	void GenerateWaitingOrder(list<list<int>> seqList);

	double CalSumOfDelay(map<int, DrivingPlan> drivingPlanDict);

	double EstSumOfDelay(map<int, DrivingPlan> drivingPlanDict);

	void TrajectoryPlanning(bool Concised, double& aTotalDelay);

	void JudgeTrajectoryType(int& typeIndex, Vehicle simuVehicle, Vehicle& leadingVehicle, Vehicle& mappingVehicle,bool& hasLeadingVehicle, bool& hasMappingVehicle);

	void TrajectoryPlanningCaseA(DrivingPlan& plan, Vehicle simuVehicle);

	void TrajectoryPlanningCaseB(DrivingPlan& plan, Vehicle simuVehicle, Vehicle leadingVehicle);

	void TrajectoryPlanningCaseC(DrivingPlan& plan, Vehicle simuVehicle, Vehicle mappingVehicle);

	void TrajectoryPlanningCaseD(DrivingPlan& plan, Vehicle simuVehicle, Vehicle leadingVehicle, Vehicle mappingVehicle);

	double GetMappingDistance(Vehicle formerVehicle, Vehicle latterVehicle);

	double GetOmegaForTurningLeft(double radius, double simuSpeed, double& calAngle);

	double GetOmegaForTurningRight(double radius, double simuSpeed, double& calAngle);

	void TrajectoryPlanningInJunction(Vehicle simuVehicle, double simuInterDistance, double simuSpeed, BPointCoordinate simuLocation, double simuAngle, double simuOmega, DrivingPlan& plan);

	double CalculateEstTime(Vehicle vehicle, Vehicle leadingVehicle, Vehicle mappingVehicle, int typeIndex);

	double LeadEstTime(Vehicle vehicle, Vehicle leadingVehicle);

	double EstimateTime(Vehicle vehicle);

	double CalculateInterTime(Vehicle vehicle);

	bool Debug_Is_Corrected(Vehicle debugVehicle);
private:


};

#endif // !_TRAJECTORY_PLANNER_
