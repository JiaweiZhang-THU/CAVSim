#pragma once
#ifndef _TRAJECTORY_PLANNING_
#define _TRAJECTORY_PLANNING_

#include "../../Object/Vehicle/Vehicle.h"
#include "CarFollowing_IDM.h"
#include "CarFollowing_Modified_IDM.h"
#include "LaneChanging.h"
#include "../Centralized/PassingIntersection.h"
#include "../Others/cls_random.h"

class TrajectoryPlanning
{
public:
	TrajectoryPlanning();
	TrajectoryPlanning(double aCarFolloingMaintainingDistance);
	~TrajectoryPlanning();

	cls_random MyRandom;

	bool IsSpeedFluctuatOnStraightLane;

	double TimeStep;
public:

	int ChoosedCarFollowingModel;
	CarFollowingIDM CarFollowingIDMModel;
	CarFollowingModifiedIDM CarFollowingModifiedIDMModel;

	LaneChanging LaneChanging;

	PassingIntersection PassingIntersectionModel;

	void SceneNormalization(list<Vehicle>& aVehiclesList, Vehicle& aVehicle);

	list<Vehicle> LeadingVehicleList(list<Vehicle> aVehiclesList, Vehicle aVehicle);

	list<Vehicle> LeadingVehicleListForVehicleOnMergingLane(list<Vehicle> aVehiclesList, Vehicle aVehicle);

	Vehicle LeadVehicle(list<Vehicle> aVehiclesList);

	list<Vehicle>  LeadingVehicleListForLCVehicleOnOwnLane(list<Vehicle> aVehiclesList, Vehicle aVehicle);

	list<Vehicle>  LeadingVehicleListForLCVehicleOnTargetLane(list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction);

	double SpeedFluctuatOnStraightLane(double aLongitudeSpeed, Vehicle aVehicle);

	bool SpeedPlanningOnMergingZone(double& CiteAgentVehicleSpeed, double& CiteAgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction);

	bool SpeedPlanningOnStraightLane(double& CiteAgentVehicleSpeed,double& CiteAgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle,int aLaneChangeAction);

	bool SpeedPlanningOnStraightLane(double& CiteAgentVehicleSpeed, double& CiteAgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction,double& CiteAgentVehicleHeadwayDistance);
	
	bool SpeedPlanningOnStraightLaneForStringStability(double& CiteAgentVehicleSpeed, double& CiteAgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle& aVehicle, int aLaneChangeAction, double aStringStabilityTesting_PacketLossRatio_PLR,double aTimeHeadway_TH);

	bool SpeedPlanningOnStraightLane_PerceptualDelay(double& CiteAgentVehicleSpeed, double& CiteAgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction);
	
	bool SpeedPlanningOnStraightLane_PerceptualDelay(double& CiteAgentVehicleSpeed, double& CiteAgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction, double& CiteAgentVehicleHeadwayDistance);
		
	bool SpeedPlanningOnIntersection(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle);

	bool IsNeedCollisionCheck;

	double CollisionCheckProcess(double aDistanceToFrontVehicle, double aVehicleSpeed_t, double aVehicleSpeed_t_plus_1, double aFrontVehicleSpeed_t);

};


#endif // !_TRAJECTORY_PLANNING_

