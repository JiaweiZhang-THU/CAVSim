#include "TrajectoryPlanning.h"
#include<iostream>
#include "TrajectoryPlanningParamters.h"

TrajectoryPlanning::TrajectoryPlanning()
{
	TrajectoryPlanning::IsSpeedFluctuatOnStraightLane = false;
	TrajectoryPlanning::TimeStep = paraTimeStepAtTrajectoryPlanning;
}

TrajectoryPlanning::TrajectoryPlanning(double aCarFolloingMaintainingDistance)
{
	TrajectoryPlanning::IsSpeedFluctuatOnStraightLane = false;
	TrajectoryPlanning::ChoosedCarFollowingModel = 0;
}

TrajectoryPlanning::~TrajectoryPlanning()
{
}

void TrajectoryPlanning::SceneNormalization(list<Vehicle>& aVehiclesList, Vehicle& aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = aVehiclesList.begin(); localVehicleIterator != aVehiclesList.end(); localVehicleIterator++)
	{
		localVehicleIterator->Location.X -= aVehicle.Location.X;
		localVehicleIterator->Location.Y -= aVehicle.Location.Y;
		localVehicleIterator->Location.X = localVehicleIterator->Location.X * cos(-aVehicle.Direction) - localVehicleIterator->Location.Y * sin(-aVehicle.Direction);
		localVehicleIterator->Location.Y = localVehicleIterator->Location.X * sin(-aVehicle.Direction) + localVehicleIterator->Location.Y * cos(-aVehicle.Direction);
		localVehicleIterator->Direction -= aVehicle.Direction;
		while (localVehicleIterator->Direction < 0)
		{
			localVehicleIterator->Direction += 2*acos(-1);
		}
		while (localVehicleIterator->Direction > (2 * acos(-1)))
		{
			localVehicleIterator->Direction -= 2 * acos(-1);
		}
	}
	
	aVehicle.Location.X -= aVehicle.Location.X;
	aVehicle.Location.Y -= aVehicle.Location.Y;
}

list<Vehicle> TrajectoryPlanning::LeadingVehicleList(list<Vehicle> aVehiclesList, Vehicle aVehicle)
{
	list<Vehicle> localLeadingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;

	TrajectoryPlanning::SceneNormalization(aVehiclesList, aVehicle);

	for (localVehicleIterator = aVehiclesList.begin(); localVehicleIterator != aVehiclesList.end(); localVehicleIterator++)
	{
		if (aVehicle.LaneId == localVehicleIterator->LaneId && localVehicleIterator->Location.X > 0 and localVehicleIterator->Id != aVehicle.Id)
		{
			localLeadingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return localLeadingVehicleList;
}


list<Vehicle> TrajectoryPlanning::LeadingVehicleListForVehicleOnMergingLane(list<Vehicle> aVehiclesList, Vehicle aVehicle)
{
	list<Vehicle> localLeadingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;

	TrajectoryPlanning::SceneNormalization(aVehiclesList, aVehicle);

	for (localVehicleIterator = aVehiclesList.begin(); localVehicleIterator != aVehiclesList.end(); localVehicleIterator++)
	{
		if (aVehicle.LaneId == localVehicleIterator->LaneId && localVehicleIterator->Location.X > 0 && aVehicle.BlockId == localVehicleIterator->BlockId)
		{
			localLeadingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return localLeadingVehicleList;
}

Vehicle TrajectoryPlanning::LeadVehicle(list<Vehicle> aVehiclesList)
{
	Vehicle localLeadingVehicle = aVehiclesList.front();
	list<Vehicle>::iterator localVehicleIterator;

	for (localVehicleIterator = aVehiclesList.begin(); localVehicleIterator != aVehiclesList.end(); localVehicleIterator++)
	{
		if (localLeadingVehicle.Location.X > localVehicleIterator->Location.X)
		{
			localLeadingVehicle = *localVehicleIterator;
		}
	}
	return localLeadingVehicle;
}

double TrajectoryPlanning::SpeedFluctuatOnStraightLane(double aLongitudeSpeed, Vehicle aVehicle)
{
	double localSpeedFluctuateProbability = 0.8;
	double localSpeedFluctuateRange = 0.7;

	double newSpeed = aLongitudeSpeed;
	if (TrajectoryPlanning::MyRandom.randomUniform(0, 1) > localSpeedFluctuateProbability && aLongitudeSpeed > aVehicle.MaxStraightSpeed * localSpeedFluctuateRange)
	{
		newSpeed = localSpeedFluctuateRange * aLongitudeSpeed;
	}

	return newSpeed;
}

bool TrajectoryPlanning::SpeedPlanningOnMergingZone(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction)
{
	double localLaturalSpeed;
	double localLongitudeSpeed;
	double localLongitudeSpeedForOwnLane;
	double localLongitudeSpeedForTargetLane;
	Vehicle localLeadingVehicle;

	bool localHaveDone;

	if (aLaneChangeAction == 0)
	{
		localLaturalSpeed = 0;

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleListForVehicleOnMergingLane(aVehiclesList, aVehicle);

		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);

			if (TrajectoryPlanning::IsNeedCollisionCheck == true)
			{
				localLongitudeSpeed = TrajectoryPlanning::CollisionCheckProcess(localLeadingVehicle.Location.X, aVehicle.Speed * cos(aVehicle.PoseAngle), localLongitudeSpeed, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle));
			}

		}

		localHaveDone = true;
	}
	else
	{
		localLaturalSpeed = TrajectoryPlanning::LaneChanging.LateralSpeed(aVehicle.DistannceFromTargetLane, aLaneChangeAction);

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleListForLCVehicleOnOwnLane(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeedForOwnLane = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeedForOwnLane = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);

			if (TrajectoryPlanning::IsNeedCollisionCheck == true)
			{
				localLongitudeSpeedForOwnLane = TrajectoryPlanning::CollisionCheckProcess(localLeadingVehicle.Location.X, aVehicle.Speed * cos(aVehicle.PoseAngle), localLongitudeSpeedForOwnLane, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle));
			}
		}

		list<Vehicle> localLeadingVehicleOnTargetLaneList = TrajectoryPlanning::LeadingVehicleListForLCVehicleOnTargetLane(aVehiclesList, aVehicle, aLaneChangeAction);
		if (localLeadingVehicleOnTargetLaneList.size() == 0)
		{
			localLongitudeSpeedForTargetLane = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleOnTargetLaneList);
			localLongitudeSpeedForTargetLane = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);

			if (TrajectoryPlanning::IsNeedCollisionCheck == true)
			{
				localLongitudeSpeedForTargetLane = TrajectoryPlanning::CollisionCheckProcess(localLeadingVehicle.Location.X, aVehicle.Speed * cos(aVehicle.PoseAngle), localLongitudeSpeedForTargetLane, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle));
			}
		}

		if (false)
		{
			localLongitudeSpeed = min(localLongitudeSpeedForOwnLane, localLongitudeSpeedForTargetLane);
		}
		else
		{
			localLongitudeSpeed = localLongitudeSpeedForOwnLane;
		}

		if (aVehicle.DistannceFromTargetLane == 0)
		{
			localHaveDone = true;
		}
		else
		{
			localHaveDone = false;
		}
	}

	AgentVehicleSpeed = pow(pow(localLaturalSpeed, 2) + pow(localLongitudeSpeed, 2), 0.5);
	AgentVehiclePoseAngle = atan(localLaturalSpeed / (localLongitudeSpeed + 0.000001));

	return localHaveDone;
}


bool TrajectoryPlanning::SpeedPlanningOnStraightLane(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction)
{

	double localLaturalSpeed = 0;
	double localLongitudeSpeed = 0;
	Vehicle localLeadingVehicle;

	bool localHaveDone;

	double localHeadwayDistance = -1.0;

	if (aLaneChangeAction == 0)
	{
		localLaturalSpeed = 0;

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle),aVehicle,localHeadwayDistance);
		}

		if (TrajectoryPlanning::IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}

		localHaveDone = true;
	}
	else
	{
		localLaturalSpeed = TrajectoryPlanning::LaneChanging.LateralSpeed(aVehicle.DistannceFromTargetLane, aLaneChangeAction);
		
		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle, localHeadwayDistance);
		}

		if (TrajectoryPlanning::IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}


		if (aVehicle.DistannceFromTargetLane == 0)
		{
			localHaveDone = true;
		}
		else
		{
			localHaveDone = false;
		}
	}

	AgentVehicleSpeed = pow(pow(localLaturalSpeed, 2) + pow(localLongitudeSpeed, 2), 0.5);
	AgentVehiclePoseAngle = atan(localLaturalSpeed / (localLongitudeSpeed+0.000001));

	return localHaveDone;
}


bool TrajectoryPlanning::SpeedPlanningOnStraightLaneForStringStability(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle& aVehicle, int aLaneChangeAction, double aStringStabilityTesting_PacketLossRatio_PLR, double aTimeHeadway_TH)
{

	double localLaturalSpeed = 0;
	double localLongitudeSpeed = 0;
	Vehicle localLeadingVehicle;

	bool localHaveDone;


	if (aLaneChangeAction == 0)
	{
		localLaturalSpeed = 0;

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDrivingForStringStability(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle, TrajectoryPlanning::TimeStep);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			double localRandom = TrajectoryPlanning::MyRandom.randomUniform(0, 1);
			if ((localRandom >= aStringStabilityTesting_PacketLossRatio_PLR) and (aVehicle.WhetherToSendRealtimeSpeed == true))
			{
				aVehicle.SpeedOfTheFrontVehicle = localLeadingVehicle.SpeedSentToFollowingVehicles * cos(localLeadingVehicle.PoseAngle);
				aVehicle.WhetherToSendRealtimeSpeed = false;
			}
			else
			{
				aVehicle.SpeedOfTheFrontVehicle = aVehicle.SpeedOfTheFrontVehicle;
			}
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowingForStringStability(
				localLeadingVehicle.Location.X, 
				aVehicle.SpeedOfTheFrontVehicle,
				aVehicle.Speed * cos(aVehicle.PoseAngle),
				aVehicle.AccelerationAtLastStep, 
				aVehicle,
				TrajectoryPlanning::TimeStep,
				aVehicle.HeadwayInIDM,
				aVehicle.Acceleration,
				aTimeHeadway_TH);

			aVehicle.HeadwayBetweenTheEgoAndFronVehicle = localLeadingVehicle.Location.X;
		}

		if (TrajectoryPlanning::IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}

		localHaveDone = true;
	}

	AgentVehicleSpeed = pow(pow(localLaturalSpeed, 2) + pow(localLongitudeSpeed, 2), 0.5);
	AgentVehiclePoseAngle = atan(localLaturalSpeed / (localLongitudeSpeed + 0.000001));

	return localHaveDone;
}


bool TrajectoryPlanning::SpeedPlanningOnStraightLane(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction, double& AgentVehicleHeadwayDistance)
{

	double localLaturalSpeed = 0;
	double localLongitudeSpeed = 0;
	Vehicle localLeadingVehicle;

	bool localHaveDone;

	double localHeadwayDistance = -1.0;

	if (aLaneChangeAction == 0)
	{
		localLaturalSpeed = 0;

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle, localHeadwayDistance);
		}

		if (IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}

		localHaveDone = true;
	}
	else
	{
		localLaturalSpeed = TrajectoryPlanning::LaneChanging.LateralSpeed(aVehicle.DistannceFromTargetLane, aLaneChangeAction);

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle, localHeadwayDistance);
		}

		if (IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}


		if (aVehicle.DistannceFromTargetLane == 0)
		{
			localHaveDone = true;
		}
		else
		{
			localHaveDone = false;
		}
	}

	AgentVehicleHeadwayDistance = localHeadwayDistance;
	AgentVehicleSpeed = pow(pow(localLaturalSpeed, 2) + pow(localLongitudeSpeed, 2), 0.5);
	AgentVehiclePoseAngle = atan(localLaturalSpeed / (localLongitudeSpeed + 0.000001));

	return localHaveDone;
}




bool TrajectoryPlanning::SpeedPlanningOnStraightLane_PerceptualDelay(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction)
{
	double localLaturalSpeed = 0;
	double localLongitudeSpeed = 0;
	Vehicle localLeadingVehicle;

	bool localHaveDone;

	double localHeadwayDistance = -1.0;

	if (aLaneChangeAction == 0)
	{
		localLaturalSpeed = 0;

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle, localHeadwayDistance);
		}

		if (TrajectoryPlanning::IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}

		localHaveDone = true;
	}
	else
	{
		localLaturalSpeed = TrajectoryPlanning::LaneChanging.LateralSpeed(aVehicle.DistannceFromTargetLane, aLaneChangeAction);

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.SpeedBeforeNSecons * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle, localHeadwayDistance);
		}

		if (TrajectoryPlanning::IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}


		if (aVehicle.DistannceFromTargetLane == 0)
		{
			localHaveDone = true;
		}
		else
		{
			localHaveDone = false;
		}
	}

	AgentVehicleSpeed = pow(pow(localLaturalSpeed, 2) + pow(localLongitudeSpeed, 2), 0.5);
	AgentVehiclePoseAngle = atan(localLaturalSpeed / (localLongitudeSpeed + 0.000001));

	return localHaveDone;
}



bool TrajectoryPlanning::SpeedPlanningOnStraightLane_PerceptualDelay(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction, double& AgentVehicleHeadwayDistance)
{

	double localLaturalSpeed = 0;
	double localLongitudeSpeed = 0;
	Vehicle localLeadingVehicle;

	bool localHaveDone;

	double localHeadwayDistance = -1.0;

	if (aLaneChangeAction == 0)
	{
		localLaturalSpeed = 0;

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.SpeedBeforeNSecons * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle, localHeadwayDistance);
		}

		if (TrajectoryPlanning::IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}

		localHaveDone = true;
	}
	else
	{
		localLaturalSpeed = TrajectoryPlanning::LaneChanging.LateralSpeed(aVehicle.DistannceFromTargetLane, aLaneChangeAction);

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);
		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle, localHeadwayDistance);
		}

		if (TrajectoryPlanning::IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}


		if (aVehicle.DistannceFromTargetLane == 0)
		{
			localHaveDone = true;
		}
		else
		{
			localHaveDone = false;
		}
	}

	AgentVehicleHeadwayDistance = localHeadwayDistance;
	AgentVehicleSpeed = pow(pow(localLaturalSpeed, 2) + pow(localLongitudeSpeed, 2), 0.5);
	AgentVehiclePoseAngle = atan(localLaturalSpeed / (localLongitudeSpeed + 0.000001));

	return localHaveDone;
}



bool TrajectoryPlanning::SpeedPlanningOnIntersection(double& AgentVehicleSpeed, double& AgentVehiclePoseAngle, list<Vehicle> aVehiclesList, Vehicle aVehicle)
{
	if (aVehicle.BlockId !=0)
	{
		double localLongitudeSpeed = 0;
		Vehicle localLeadingVehicle;

		list<Vehicle> localLeadingVehicleList = TrajectoryPlanning::LeadingVehicleList(aVehiclesList, aVehicle);

		if (localLeadingVehicleList.size() == 0)
		{
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingModifiedIDMModel.FreeDriving(aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}
		else
		{
			localLeadingVehicle = TrajectoryPlanning::LeadVehicle(localLeadingVehicleList);
			localLongitudeSpeed = TrajectoryPlanning::CarFollowingModifiedIDMModel.AgentBasedCarFollowing(localLeadingVehicle.Location.X, localLeadingVehicle.Speed * cos(localLeadingVehicle.PoseAngle), aVehicle.Speed * cos(aVehicle.PoseAngle), aVehicle);
		}

		if (IsSpeedFluctuatOnStraightLane == true)
		{
			localLongitudeSpeed = TrajectoryPlanning::SpeedFluctuatOnStraightLane(localLongitudeSpeed, aVehicle);
		}
		AgentVehicleSpeed = localLongitudeSpeed;
		AgentVehiclePoseAngle = 0;
		return true;
	}
	else
	{
		TrajectoryPlanning::PassingIntersectionModel.SpeedPlanning(AgentVehicleSpeed, AgentVehiclePoseAngle, aVehiclesList, aVehicle);
		return true;
	}
}


double TrajectoryPlanning::CollisionCheckProcess(double aDistanceToFrontVehicle, double aVehicleSpeed_t, double aVehicleSpeed_t_plus_1, double aFrontVehicleSpeed_t)
{
	double localDeltaX = paraCollisionCheck_a_1 * paraCollisionCheck_L_car + paraCollisionCheck_a_2 * aVehicleSpeed_t;
	if (aDistanceToFrontVehicle <= localDeltaX)
	{
		if (aVehicleSpeed_t_plus_1 <= aFrontVehicleSpeed_t - paraCollisionCheck_v_cc)
		{
			return aVehicleSpeed_t_plus_1;
		}
		else
		{
			return fmax(0, aFrontVehicleSpeed_t - paraCollisionCheck_v_cc);
		}
	}
	else
	{
		return aVehicleSpeed_t_plus_1;
	}
}

list<Vehicle> TrajectoryPlanning::LeadingVehicleListForLCVehicleOnOwnLane(list<Vehicle> aVehiclesList, Vehicle aVehicle)
{
	list<Vehicle> localLeadingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;

	TrajectoryPlanning::SceneNormalization(aVehiclesList, aVehicle);

	for (localVehicleIterator = aVehiclesList.begin(); localVehicleIterator != aVehiclesList.end(); localVehicleIterator++)
	{
		if (aVehicle.LaneId == localVehicleIterator->LaneId && localVehicleIterator->Location.X > 0)
		{
			localLeadingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return localLeadingVehicleList;
}


list<Vehicle> TrajectoryPlanning::LeadingVehicleListForLCVehicleOnTargetLane(list<Vehicle> aVehiclesList, Vehicle aVehicle, int aLaneChangeAction)
{
	list<Vehicle> localLeadingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;

	TrajectoryPlanning::SceneNormalization(aVehiclesList, aVehicle);

	for (localVehicleIterator = aVehiclesList.begin(); localVehicleIterator != aVehiclesList.end(); localVehicleIterator++)
	{
		if (aVehicle.LaneId + aLaneChangeAction == localVehicleIterator->LaneId && localVehicleIterator->Location.X > 0)
		{
			localLeadingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return localLeadingVehicleList;
}
