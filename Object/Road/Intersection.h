#ifndef _INTERSECTION_
#define _INTERSECTION_

#include<set>
#include "BConflictSubzone.h"
#include "StraightLaneBlock.h"
#include "..\Vehicle\Vehicle.h"

const double PI = acos(-1);

class Intersection: public BRectangle
{
public:
	int NumOfOneWayLanes;

	double LaneWidth;

	double IntersectionStraightBlockLength;

	BPointCoordinate RotationCoordiante(BPointCoordinate aOriginalCoordinates, double aTheta);

	int NumOfConflictSubzone;

	BConflictSubzone ConflictSubzone[20][20];
	int ConflictSubzonesLastPassedVehicleID[20][20];
	double ConflictSubzonesLastPassedVehicleArrivalTime[20][20];
	
	void SetConflictSubzone();
	void SetConflictSubzoneNet();
	void SetDriveInAndOutBool();

	BConflictSubzone FindConflictSubzone(double aX, double aY);
	void VehicleTrajectoryCoverageConflictSubzones(Vehicle& aVehicle);

	double GetSteerRadius(int aLaneId, int aSteeringType);
	BPointCoordinate GetSteerCenter(int aBlockId, int aSteeringType);

	list<Vehicle> VehicleListInCrossingArea;
	void ClearVehicleInCrossingArea();
	void UpdateVehicleInCrossingArea(Vehicle aVehicle);
	void AddVehicleInCrossingArea(Vehicle aVehicle);
	void RemoveVehicleInCrossingArea(Vehicle aVehicle);

	list<Vehicle> VehicleList;
	void ClearVehicle();
	void UpdateVehicle(Vehicle aVehicle);
	void AddVehicle(Vehicle aVehicle);
	void RemoveVehicle(Vehicle aVehicle);
	list<Vehicle> DrivingInVehicles[4*10]; 
	int StraightLaneVehiclesAN[8];
	int StraightLaneVehiclesAnExiting[8];
	int StraightLaneVehiclesRealTimeNum[8];
	int PassingIntersectionVehiclesNum;
	int CurrentMovingInIntersection;
	void InitialStatisticalVariable();

	StraightLaneBlock StraightLaneInterface[8];
	void SetStraightLaneInterface();

	BRectangle IntersectionNineRectangle[9];
	void SetIntersectionNineRectangle();

	void SetIntersectionNet();

	bool IsInIntersection(BPointCoordinate aLocation);
	bool IsInIntersection(double aLocationX,double aLocationY);

	bool IsInConflictZone(double aLocationX, double aLocationY);

	int WhichAreaInIntersection(BPointCoordinate aLocation);
	BNode DrivingInPlace(int aBlockId, int aLaneUnitId);
	BNode DrivingOutPlace(int aBlokId, int aLaneUnitId);

	int SteerTargetBlock(Vehicle aVehicle, int aSteerType);

	TurnDirectionType WhichTurnDirectionType(int EnteringBlockId, int TargetBlockId);

	list<BNode> EnteringVehicleInitialPositonAndPoseList;
	void SetEnteringVehicleInitialPositonAndBoseList();

	LaneUnit* FindLaneUnitPointer(int BlockId, int LaneUnitId);

	Intersection();
	Intersection(int aNumOfOneWayLanes, BPointCoordinate aLocation, double aDirection);
	Intersection(int aNumOfOneWayLanes, BPointCoordinate aLocation, double aDirection,double aIntersectionStraightBlockLength);

	string GetName();
	

};

#endif // !_INTERSECTION_


