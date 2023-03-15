#ifndef _Vehicle_
#define _Vehicle_

#pragma once
#include<list>
#include<iostream>

#include "..\Road\BNode.h"
#include "..\Road\BRectangle.h"

class LaneUnit;
class StraightLane;
class Intersection;

class DrivingPlan;

using namespace std;
enum VehiclePlanState
{
	Free = 0,
	Allocated = 1,
	Locaked = 2
};

enum VehicleInIntersectionRadius
{
	Inner= 0,
	Middle = 1,
	Outter = 2

};

enum TurnDirectionType
{
	NSRight = 0,
	NSStraight = 1,
	NSLeft = 2,

	WERight = 3,
	WEStraight = 4,
	WELeft = 5
};

class Vehicle: public BNode
{
public:
	double Length;

	double Width;

	int Type;

	double MaxStraightSpeed;

	double MinStraightSpeed;

	double MaxStraightAccel;

	double MinStraightAccel;

	double MaxSteerSpeed;

	double MaxAngularAccel;

	double MaxAngular;

	double Speed;
	double SpeedAtLastStep;

	double SpeedSentToFollowingVehicles;
	double WhetherToSendRealtimeSpeed;
	double SpeedOfTheFrontVehicle;

	void UpdateSpeedSentToFollowingVehicles();

	double Acceleration;
	double AccelerationAtLastStep;

	void UpdateLassSpeedSetPastNSeconds(int aSimulatingStep);

	list<double> SpeedSetPastNSecondsList;

	double SpeedBeforeNSecons;

	double AngularSpeed;

	double PoseAngle;

	double TimeStep; 

	double HeadwayDistance;

	double HeadwayBetweenTheEgoAndFronVehicle;

	double HeadwayInIDM;

	bool IsCollideWithOtherVehicles(list<Vehicle> aSurroundingVehicles);

	BRectangle VehicleRectangle;

	bool IsCollide;

	int RoadId;

	int BlockId;

	int LaneId;

	LaneUnit *LaneUnitPointer;

	int RoadBlockType; 

	StraightLane* StraightLanePointer;

	Intersection* IntersectionPointer;

	int LastLaneChangeAction;

	double DistannceFromTargetLane;

	void UpdateDistannceFromTargetLane(double DeltaTime);

	int SteeringType;

	TurnDirectionType SteeringDriectionType; 

	double SteeringRadius;

	BPointCoordinate SteeringCenter;

	BNode EnteringNode;

	BNode TargetNode; 

	int EnteringBlockId;

	int TargetBlockId; 

	double TimeCostToTargetNede;

	int TrajectoryCoverageConflictSubzonesArraySize;

	int TrajectoryCoverageConflictSubzonesArray[40]; 

	double MinimumArrivalTimeToConflictSubzones[40];

	double AssignedArrivalTimeToConflictSubzones[40];

	double AssignedTimeToArriveConfliceZone;

	double DeltaTimeBetweenTwoConflictSubzone;

	double MinimumArrivalTime;

	int FromNodeId;

	int ToNodeId;

	VehiclePlanState PlanState;
	
	VehicleInIntersectionRadius InIntersectionRadius;

	int WaitingOrder = 0;

	int FirstLaneId = 0;

	int SecondLaneId = 0;

	double LeftDistance;

	double LeftLaneDistance;

	double LeftInterDistance;

	double NoiseInDistance;

	double QueueingIndex;

	double EstTime;

	double InterTime;

	double TotalEst;

	string StrategyMode;

	int HeadId;

	double HeadDistance;

	double HeadSpeed;

	int ConflictId;

	list<double> LeadDistList;

	int PassingTime;

	int DRPassingOrder = -1;

	int rulePassingOrder = -1;

	DrivingPlan* MyDrivingPlanPointer;

	double IDM_Parameters_b;

	double IDM_Parameters_T0;

	void UpdatePerceptionSurroundingIntersectionVehicles(list<Vehicle> aPerceptionSurroundingVehiclesList, Vehicle& aVehicle);

	void SceneNormalization(list<Vehicle>& aVehiclesList, Vehicle& aVehicle);

	list<Vehicle> MiddleFollowingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList);

	list<Vehicle> LeftFollowingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList);

	list<Vehicle> RightFollowingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList);

	list<Vehicle> MiddleLeadingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList);

	list<Vehicle> LeftLeadingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList);

	list<Vehicle> RightLeadingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList);

	Vehicle FindFollowingVehicle(list<Vehicle> aFollowingVehicleList);

	Vehicle FindLeadingVehicle(list<Vehicle> aFollowingVehicleList);

	bool CanMoveForward;

	double ShouldBornTime;

	double BornTime;

	double Arrival;

	double DepartureTime;

	list<double> SpeedList;
	
	list<double> AngularSpeedList;

	list<double> XLocationList;

	list<double> YLocationList;

	double TravelDistance;

	list<double> RecodeSpeedList;

	double TimeOfEnteringIntersectionCircleControlZone;

	double TimeOfEnteringIntersectionConflictZone;

	double DelayOfPassingIntersection;

	double EnergyConsumpingOfPassingIntersection;

	double FuelConsumptionOfPassingIntersection;

	bool WhetherTheInitialPositionIsOnTheMainRoad;

	void NewList();

	void ClearList();

	void AddList(double _Speed, double _AngularSpeed, double _XLocation, double _YLocation);
	
	Vehicle();

	Vehicle(BPointCoordinate aInitialLocation, double aInitialPoseAngle);

	Vehicle(BPointCoordinate aInitialLocation, double aInitialPoseAngle, double aDirection);

	string GetName();

	void ResetIDMParameters();

	void UpdateVehicleLocation(double DeltaTime);

	void UpdateVehicleLocationForStringStability(double DeltaTime);

	void UpdateVehicleRectangularBox();

	void UpdateLeftDistance(double DeltaTime);
	
	double Normal();

	int PathStartNode;

	int PathEndNode;

	double PathDistance;

	int PathNodeNum;

	int PathNodeList[100];

	int RealtimeLastLastNode;

	int RealtimeLastNode;

	int RealtimeNextNode;

	int RealtimeNextNodeIndex;

};

#endif // !_Vehicle_