#include"TrajectoryPlanner.h"
#include"TrajectoryPlannerParameters.h"

TrajectoryPlanner::TrajectoryPlanner()
{
	TrajectoryPlanner::DeltaTime = paraDeltaTime;
	TrajectoryPlanner::MaintainingDistance = paraMaintainingDistance1;
}

TrajectoryPlanner::TrajectoryPlanner(
	double aSimulator_TimeCount,
	double aLaneNum,
	double aOuterRadius,
	list<Vehicle> aDrivingInVehiclesListArray[4 * 10],
	list<Vehicle> aLastLockingList,
	map<int, Vehicle> aSimuVehicleDict,
	map<int, DrivingPlan> aPlanBasedApproach_DrivingPlanDict,
	double aConflictSubzonesLastPassedVehicleArrivalTime[20][20],
	list<int> aPassingOrderList
)
{
	TrajectoryPlanner::DeltaTime = paraDeltaTime;
	TrajectoryPlanner::Simulator_TimeCount = aSimulator_TimeCount;
	TrajectoryPlanner::LaneNum = aLaneNum;
	TrajectoryPlanner::LaneWidth = paraLaneWidth3;
	TrajectoryPlanner::OuterRadius = aOuterRadius;
	TrajectoryPlanner::MaintainingDistance = paraMaintainingDistance1;

	for (int i = 0; i < 4 * TrajectoryPlanner::LaneNum; i++)
	{
		TrajectoryPlanner::DrivingInVehiclesListArray[i].clear();
		for (list<Vehicle>::iterator VIter = aDrivingInVehiclesListArray[i].begin(); VIter != aDrivingInVehiclesListArray[i].end(); VIter++)
		{
			TrajectoryPlanner::DrivingInVehiclesListArray[i].push_back(*VIter);
		}
	}

	TrajectoryPlanner::LastLockingList.clear();
	for (list<Vehicle>::iterator VIter = aLastLockingList.begin(); VIter != aLastLockingList.end(); VIter++)
	{
		TrajectoryPlanner::LastLockingList.push_back(*VIter);
	}

	TrajectoryPlanner::SimuVehicleDict.clear();
	for (map<int, Vehicle>::iterator VIter = aSimuVehicleDict.begin(); VIter != aSimuVehicleDict.end(); VIter++)
	{
		TrajectoryPlanner::SimuVehicleDict[VIter->first] = VIter->second;
	}

	TrajectoryPlanner::PlanBasedApproach_DrivingPlanDict.clear();
	for (map<int, DrivingPlan>::iterator DPIter = aPlanBasedApproach_DrivingPlanDict.begin(); DPIter != aPlanBasedApproach_DrivingPlanDict.end(); DPIter++)
	{
		TrajectoryPlanner::PlanBasedApproach_DrivingPlanDict[DPIter->first] = DPIter->second;
	}

	for (int i = 0; i < 2 * TrajectoryPlanner::LaneNum; i++)
	{
		for (int j = 0; j < 2 * TrajectoryPlanner::LaneNum; j++)
		{
			TrajectoryPlanner::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = aConflictSubzonesLastPassedVehicleArrivalTime[i][j];
		}
	}

	TrajectoryPlanner::PassingOrderList = aPassingOrderList;
}


TrajectoryPlanner::~TrajectoryPlanner()
{
}

void TrajectoryPlanner::Init()
{
	TrajectoryPlanner::PlanningVehicleList = *new list<Vehicle>();
}

void TrajectoryPlanner::Load(map<int, DrivingPlan> aDrivingPlanDict, list<list<int>> aSeqList, map<int, Vehicle> aSimuVehicleDict)
{
	TrajectoryPlanner::_drivingPlanDict = aDrivingPlanDict;
	TrajectoryPlanner::PlanningVehicleList = *new list<Vehicle>();

	list<list<int>>::iterator localListListIntIterator;
	list<int>::iterator localListIntIterator;
	
	for (localListIntIterator = TrajectoryPlanner::PassingOrderList.begin(); localListIntIterator != TrajectoryPlanner::PassingOrderList.end(); localListIntIterator++)
	{
		Vehicle simuVeh = aSimuVehicleDict[*localListIntIterator];
		TrajectoryPlanner::PlanningVehicleList.push_back(simuVeh);
		TrajectoryPlanner::BindingPlanWithVehicle(simuVeh.Id);

	}

	TrajectoryPlanner::GenerateWaitingOrder(aSeqList);
}

void TrajectoryPlanner::BindingPlanWithVehicle(int vehicleId)
{
	DrivingPlan plan = *new DrivingPlan();
	TrajectoryPlanner::_drivingPlanDict[vehicleId] = plan;
}


void TrajectoryPlanner::Run(double& calDelay, double& estDelay, bool Concised,double& aTotalDelay)
{
	
	TrajectoryPlanner::TrajectoryPlanning(Concised,aTotalDelay);

}

void TrajectoryPlanner::GenerateWaitingOrder(list<list<int>> seqList)
{
	if (TrajectoryPlanner::LastLockingList.size() != 0)
	{
		int lastLockingWaitingOrder = 0;
		for (list<Vehicle>::iterator VIter = TrajectoryPlanner::LastLockingList.begin(); VIter != TrajectoryPlanner::LastLockingList.end(); VIter++)
		{
			if (VIter->WaitingOrder / 10 > lastLockingWaitingOrder)
			{
				lastLockingWaitingOrder = VIter->WaitingOrder / 10;
			}
		}

		int sameGroupOffset = 1;
		bool isSafeToFork = true;
		int lastWaitingOrder = 0;

		for (list<int>::iterator VehicleIdIter = seqList.begin()->begin(); VehicleIdIter != seqList.begin()->end(); VehicleIdIter++)
		{
			Vehicle vehicleA = TrajectoryPlanner::SimuVehicleDict[*VehicleIdIter];
			if (isSafeToFork)
			{
				for (list<Vehicle>::iterator simuVeh = TrajectoryPlanner::LastLockingList.begin(); simuVeh != TrajectoryPlanner::LastLockingList.end(); simuVeh++)
				{
					if (TrajectoryPlanner::MyCollisionDetection.IsTwoVehicleCollision(vehicleA, *simuVeh) == true)
					{
						isSafeToFork = false;
						break;
					}
				}
			}

			if (!isSafeToFork)
			{
				lastWaitingOrder = lastLockingWaitingOrder * 10 + 9 + sameGroupOffset;
				TrajectoryPlanner::_drivingPlanDict[*VehicleIdIter].WaitingOrder = lastWaitingOrder;
			}
			else
			{
				lastWaitingOrder = lastLockingWaitingOrder * 10 + sameGroupOffset;
				TrajectoryPlanner::_drivingPlanDict[*VehicleIdIter].WaitingOrder = lastWaitingOrder;
			}
			sameGroupOffset++;
		}

		int lastIndex = lastWaitingOrder;
		int startGroupIndex = lastIndex / 10 + 1;
		int groupDelta = 0;
		for (list<list<int>>::iterator ListListIntIter = seqList.begin(); ListListIntIter != seqList.end(); ListListIntIter++)
		{
			int index = 0;
			for (list<int>::iterator ListIntIter = ListListIntIter->begin(); ListIntIter != ListListIntIter->end(); ListIntIter++)
			{
				TrajectoryPlanner::_drivingPlanDict[*ListIntIter].WaitingOrder = (startGroupIndex + groupDelta) * 10 + index;
				index++;
			}
			groupDelta++;
		}
	}
	else
	{
		int groupIndex = 0;
		for (list<list<int>>::iterator ListListIntIter = seqList.begin(); ListListIntIter != seqList.end(); ListListIntIter++)
		{
			int index = 0;
			for (list<int>::iterator ListIntIter = ListListIntIter->begin(); ListIntIter != ListListIntIter->end(); ListIntIter++)
			{
				TrajectoryPlanner::_drivingPlanDict[*ListIntIter].WaitingOrder = groupIndex * 10 + index;
				index++;
			}
			groupIndex++;
		}
	}
}

double TrajectoryPlanner::CalSumOfDelay(map<int, DrivingPlan> drivingPlanDict)
{
	double delay = 0;
	for (map<int, DrivingPlan>::iterator mapIter = drivingPlanDict.begin(); mapIter != drivingPlanDict.end(); mapIter++)
	{
		DrivingPlan drivePlan = TrajectoryPlanner::_drivingPlanDict[mapIter->first];
		Vehicle vehicle = TrajectoryPlanner::SimuVehicleDict[mapIter->first];
		if (vehicle.SteeringType == 0)
		{
			delay += drivePlan.GetCount() * TrajectoryPlanner::DeltaTime + Simulator_TimeCount - 
				vehicle.BornTime + 
				(TrajectoryPlanner::OuterRadius - TrajectoryPlanner::LaneNum * TrajectoryPlanner::LaneWidth) / vehicle.MaxStraightSpeed;

		}
		else
		{
			delay += drivePlan.GetCount() * TrajectoryPlanner::DeltaTime + TrajectoryPlanner::Simulator_TimeCount - 
				vehicle.BornTime + 
				(TrajectoryPlanner::OuterRadius - TrajectoryPlanner::LaneNum * TrajectoryPlanner::LaneWidth - 
					(vehicle.MaxStraightSpeed * vehicle.MaxStraightSpeed - 
						vehicle.MaxSteerSpeed * vehicle.MaxSteerSpeed) / 2 / vehicle.MaxStraightAccel) / 
				vehicle.MaxStraightSpeed + 
				(vehicle.MaxStraightSpeed - vehicle.MaxSteerSpeed) / vehicle.MaxStraightAccel;

		}
	}

	return delay / drivingPlanDict.size();
}

double TrajectoryPlanner::EstSumOfDelay(map<int, DrivingPlan> drivingPlanDict)
{
	double delay = 0;
	for (map<int, DrivingPlan>::iterator mapIter = drivingPlanDict.begin(); mapIter != drivingPlanDict.end(); mapIter++)
	{
		Vehicle vehicle = TrajectoryPlanner::SimuVehicleDict[mapIter->first];
		delay += vehicle.TotalEst;
		if (vehicle.SteeringType == 0)
		{
			delay += TrajectoryPlanner::Simulator_TimeCount - 
				vehicle.BornTime + 
				(TrajectoryPlanner::OuterRadius - TrajectoryPlanner::LaneNum * TrajectoryPlanner::LaneWidth) / vehicle.MaxStraightSpeed;
		}
		else
		{
			delay += TrajectoryPlanner::Simulator_TimeCount - 
				vehicle.BornTime + 
				(TrajectoryPlanner::OuterRadius - TrajectoryPlanner::LaneNum * TrajectoryPlanner::LaneWidth - 
					(vehicle.MaxStraightSpeed * vehicle.MaxStraightSpeed - 
						vehicle.MaxSteerSpeed * vehicle.MaxSteerSpeed) / 2 / vehicle.MaxStraightAccel) / vehicle.MaxStraightSpeed + 
				(vehicle.MaxStraightSpeed - vehicle.MaxSteerSpeed) / vehicle.MaxStraightAccel;
		}
	}
	return delay / drivingPlanDict.size();
}

void TrajectoryPlanner::SceneNormalization(list<Vehicle>& aVehiclesList, Vehicle& aVehicle)
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
			localVehicleIterator->Direction += 2 * acos(-1);
		}
		while (localVehicleIterator->Direction > (2 * acos(-1)))
		{
			localVehicleIterator->Direction -= 2 * acos(-1);
		}
	}

	aVehicle.Location.X -= aVehicle.Location.X;
	aVehicle.Location.Y -= aVehicle.Location.Y;
}

list<Vehicle> TrajectoryPlanner::LeadingVehicleList(list<Vehicle> aVehiclesList, Vehicle aVehicle)
{
	list<Vehicle> localLeadingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;

	TrajectoryPlanner::SceneNormalization(aVehiclesList, aVehicle);

	for (localVehicleIterator = aVehiclesList.begin(); localVehicleIterator != aVehiclesList.end(); localVehicleIterator++)
	{
		if (aVehicle.LaneId == localVehicleIterator->LaneId && localVehicleIterator->Location.X > 0)
		{
			localLeadingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return localLeadingVehicleList;
}

Vehicle TrajectoryPlanner::LeadVehicle(list<Vehicle> aVehiclesList)
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

void TrajectoryPlanner::TrajectoryPlanning(bool Concised, double& aTotalDelay)
{
	aTotalDelay = TrajectoryPlanner::ArrivalTimeCalculByIterativeSolution();
	
	for (list<Vehicle>::iterator simuVehicle = TrajectoryPlanner::PlanningVehicleList.begin(); simuVehicle != TrajectoryPlanner::PlanningVehicleList.end(); simuVehicle++)
	{
		DrivingPlan& plan = TrajectoryPlanner::_drivingPlanDict[simuVehicle->Id];
		TrajectoryPlanner::AnalyticalTrajectoryPLanning(plan, *simuVehicle);
	}
}

double TrajectoryPlanner::ArrivalTimeCalculByIterativeSolution()
{
	ArrivalTime* MyArrivalTime = new ArrivalTime(
		TrajectoryPlanner::ConflictSubzonesLastPassedVehicleArrivalTime, 
		int(TrajectoryPlanner::LaneNum)
	);

	double JTotalDelay = MyArrivalTime->PassingOrderToTrajectoryInterPretaton(
		TrajectoryPlanner::PassingOrderList, 
		TrajectoryPlanner::SimuVehicleDict, 
		TrajectoryPlanner::Simulator_TimeCount*TrajectoryPlanner::DeltaTime
	);

	for (list<Vehicle>::iterator simuVehicle = TrajectoryPlanner::PlanningVehicleList.begin(); simuVehicle != TrajectoryPlanner::PlanningVehicleList.end(); simuVehicle++)
	{
		Vehicle& localVehicle = TrajectoryPlanner::SimuVehicleDict[simuVehicle->Id];
		simuVehicle->AssignedTimeToArriveConfliceZone = localVehicle.AssignedTimeToArriveConfliceZone;
	}

	for (int i = 0; i < 2 * TrajectoryPlanner::LaneNum; i++)
	{
		for (int j = 0; j < 2 * TrajectoryPlanner::LaneNum; j++)
		{
			TrajectoryPlanner::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = MyArrivalTime->LargestArrivalTimeInConflictSubzones[i][j];
		}
	}

	delete MyArrivalTime;

	return JTotalDelay;
}

void TrajectoryPlanner::AnalyticalTrajectoryPLanning(DrivingPlan& plan, Vehicle simuVehicle)
{
	double simuLaneDistance = simuVehicle.LeftLaneDistance;
	double simuInterDistance = simuVehicle.LeftInterDistance;
	double delta = simuInterDistance;

	BPointCoordinate simuLocation = *new BPointCoordinate();
	simuLocation.X = simuVehicle.Location.X;
	simuLocation.Y = simuVehicle.Location.Y;

	double simuAngle = simuVehicle.PoseAngle;

	double simuOmega = 0;

	double newLocation = 0;
	double newVelocity = 0;
	double energy = 0;

	double oldLocation = simuLaneDistance;
	double oldVelocity = simuVehicle.Speed;
	double tf = simuVehicle.AssignedTimeToArriveConfliceZone - TrajectoryPlanner::Simulator_TimeCount * TrajectoryPlanner::DeltaTime;
	
	TrajectoryPlanner::MyOptimalController.Run(plan, simuVehicle, tf);
	
	TrajectoryPlanner::TrajectoryPlanningInMergeZone(simuVehicle, simuInterDistance, simuLocation, simuAngle, simuOmega, plan);
}

void TrajectoryPlanner::JudgeTrajectoryType(int& typeIndex, Vehicle simuVehicle, Vehicle& leadingVehicle, Vehicle& mappingVehicle, bool& hasLeadingVehicle, bool& hasMappingVehicle)
{

	list<Vehicle> localLeadingVehicleList = TrajectoryPlanner::LeadingVehicleList(
		TrajectoryPlanner::DrivingInVehiclesListArray[(int)(((simuVehicle.BlockId - 1) / 2) * TrajectoryPlanner::LaneNum + simuVehicle.LaneId - 1)], simuVehicle
	);
	if (localLeadingVehicleList.size() == 0)
	{
		hasLeadingVehicle = false;
	}
	else
	{
		hasLeadingVehicle = true;
		leadingVehicle = TrajectoryPlanner::LeadVehicle(localLeadingVehicleList);
	}

	double longestWaitingTime = -100;

	DrivingPlan plan = TrajectoryPlanner::_drivingPlanDict[simuVehicle.Id];

	if (TrajectoryPlanner::LastLockingList.size() != 0)
	{
		for (list<Vehicle>::iterator possVehicle = TrajectoryPlanner::LastLockingList.begin(); possVehicle != TrajectoryPlanner::LastLockingList.end(); possVehicle++)
		{
			DrivingPlan hisPlan = TrajectoryPlanner::_drivingPlanDict[possVehicle->Id];
			if (possVehicle->FirstLaneId == simuVehicle.FirstLaneId)
			{
				continue;
			}
			if (possVehicle->WaitingOrder / 10 < plan.WaitingOrder / 10 && longestWaitingTime < hisPlan.GetCount())
			{
				mappingVehicle = *possVehicle;
				hasMappingVehicle = true;
				longestWaitingTime = hisPlan.GetCount();
			}
		}
		int lastWaitingGroupIndex = plan.WaitingOrder / 10 - 1;
		for (list<Vehicle>::iterator possVehicle = TrajectoryPlanner::PlanningVehicleList.begin(); possVehicle != TrajectoryPlanner::PlanningVehicleList.end(); possVehicle++)
		{
			DrivingPlan hisPlan = TrajectoryPlanner::_drivingPlanDict[possVehicle->Id];
			if (hisPlan.WaitingOrder / 10 <= lastWaitingGroupIndex && possVehicle->FirstLaneId != simuVehicle.FirstLaneId)
			{
				if (longestWaitingTime < hisPlan.GetCount())
				{
					mappingVehicle = *possVehicle;
					hasMappingVehicle = true;
					longestWaitingTime = hisPlan.GetCount();
				}
			}
		}
	}
	else
	{
		int lastWaitingGroupIndex = plan.WaitingOrder / 10 - 1;

		for (list<Vehicle>::iterator possVehicle = TrajectoryPlanner::PlanningVehicleList.begin(); possVehicle != TrajectoryPlanner::PlanningVehicleList.end(); possVehicle++)
		{
			DrivingPlan hisPlan = TrajectoryPlanner::_drivingPlanDict[possVehicle->Id];
			if (hisPlan.WaitingOrder / 10 <= lastWaitingGroupIndex && possVehicle->FirstLaneId != simuVehicle.FirstLaneId)
			{
				if (longestWaitingTime < hisPlan.GetCount())
				{
					mappingVehicle = *possVehicle;
					hasMappingVehicle = true;
					longestWaitingTime = hisPlan.GetCount();
				}
			}
		}
	}

	if (hasMappingVehicle == false)
	{
		if (hasLeadingVehicle == false)
		{
			typeIndex = 1;
		}
		else
		{
			typeIndex = 2;
		}
	}
	else
	{
		if (hasLeadingVehicle == false)
		{
			typeIndex = 3;
		}
		else
		{
			typeIndex = 4;
		}
	}
}


void TrajectoryPlanner::TrajectoryPlanningCaseA(DrivingPlan& plan, Vehicle simuVehicle)
{
	double simuLaneDistance = simuVehicle.LeftLaneDistance;
	double simuInterDistance = simuVehicle.LeftInterDistance;
	double delta = simuInterDistance;

	BPointCoordinate simuLocation = *new BPointCoordinate();
	simuLocation.X = simuVehicle.Location.X;
	simuLocation.Y = simuVehicle.Location.Y;

	double simuAngle = simuVehicle.PoseAngle;
	double simuSpeed = simuVehicle.Speed;
	double simuOmega = 0;

	if (simuVehicle.SteeringType != 0)
	{
		while (simuLaneDistance > 0)
		{
			if ((simuSpeed * simuSpeed - simuVehicle.MaxSteerSpeed) / 2 / simuVehicle.MaxStraightAccel > simuLaneDistance)
			{
				simuSpeed = fmax(simuVehicle.MaxSteerSpeed, simuSpeed - simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}
			else
			{
				simuSpeed = fmin(simuVehicle.MaxStraightSpeed, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}
			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;

			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);
			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			plan.DistanceList.push_back(simuLaneDistance + delta);
		}
	}
	else
	{
		while (simuLaneDistance > 0)
		{
			simuSpeed = fmin(simuVehicle.MaxStraightSpeed,
				simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);
			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			plan.DistanceList.push_back(simuLaneDistance + delta);
		}

	}
	
	TrajectoryPlanner::TrajectoryPlanningInJunction(simuVehicle, simuInterDistance, simuSpeed, simuLocation, simuAngle, simuOmega, plan);

}

void TrajectoryPlanner::TrajectoryPlanningCaseB(DrivingPlan& plan, Vehicle simuVehicle, Vehicle leadingVehicle)
{
	int index = 0;

	double simuLaneDistance = simuVehicle.LeftLaneDistance;
	double simuInterDistance = simuVehicle.LeftInterDistance;
	double delta = simuInterDistance;
	double leadLaneDistance = leadingVehicle.LeftLaneDistance;

	BPointCoordinate simuLocation = *new BPointCoordinate();
	simuLocation.X = simuVehicle.Location.X;
	simuLocation.Y = simuVehicle.Location.Y;

	double simuAngle = simuVehicle.PoseAngle;
	double simuSpeed = simuVehicle.Speed;
	double simuOmega = 0;

	list<double> leadSpeedList;
	if (leadingVehicle.PlanState == VehiclePlanState::Allocated)
	{
		leadSpeedList = TrajectoryPlanner::_drivingPlanDict[leadingVehicle.Id].SpeedList;
	}
	else
	{
		leadSpeedList = TrajectoryPlanner::PlanBasedApproach_DrivingPlanDict[leadingVehicle.Id].SpeedList;
	}
	list<double>::iterator leadSpeedList_Count = leadSpeedList.begin();

	if (simuVehicle.SteeringType != 0)
	{
		while (leadLaneDistance > 0)
		{
			double leadDist = simuLaneDistance - leadLaneDistance;
			plan.LeadDistList.push_back(leadDist);
			double leadSpeed = *leadSpeedList_Count;
			leadSpeedList_Count++;

			double upperBoundSpeed = simuSpeed;
			
			TrajectoryPlanner::MyFollowingBehaviors.AgentBasedFollowingBehavior(leadingVehicle.NoiseInDistance, leadDist, leadSpeed, upperBoundSpeed, simuVehicle);

			if ((simuSpeed * simuSpeed - simuVehicle.MaxSteerSpeed * simuVehicle.MaxSteerSpeed) / 2 / simuVehicle.MaxStraightAccel >
				simuLaneDistance)
			{
				simuSpeed = fmax(simuVehicle.MaxSteerSpeed, simuSpeed - simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}
			else
			{

				simuSpeed = fmin(upperBoundSpeed, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			leadLaneDistance -= TrajectoryPlanner::DeltaTime * leadSpeed;
			index++;

			plan.DistanceList.push_back(simuLaneDistance + delta);

		}
		while (simuLaneDistance > 0)
		{
			if ((simuSpeed * simuSpeed - simuVehicle.MaxSteerSpeed * simuVehicle.MaxSteerSpeed) / 2 / simuVehicle.MaxStraightAccel >
				simuLaneDistance)
			{
				simuSpeed = fmax(simuVehicle.MaxSteerSpeed,
					simuSpeed - simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}
			else
			{
				simuSpeed = fmin(simuVehicle.MaxStraightSpeed,
					simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;

			plan.DistanceList.push_back(simuLaneDistance + delta);
		}
	}
	else
	{
		while (leadLaneDistance>0)
		{
			double leadDist = simuLaneDistance - leadLaneDistance;
			plan.LeadDistList.push_back(leadDist);
			double leadSpeed = *leadSpeedList_Count;
			leadSpeedList_Count++;

			double  upperBoundSpeed = simuSpeed;
			TrajectoryPlanner::MyFollowingBehaviors.AgentBasedFollowingBehavior(leadingVehicle.NoiseInDistance, leadDist, leadSpeed, upperBoundSpeed,simuVehicle);

			simuSpeed = fmin(upperBoundSpeed, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			leadLaneDistance -= TrajectoryPlanner::DeltaTime * leadSpeed;
			index++;

			plan.DistanceList.push_back(simuLaneDistance + delta);

		}
		while (simuLaneDistance > 0)
		{
			simuSpeed = fmin(simuVehicle.MaxStraightSpeed,
				simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			plan.DistanceList.push_back(simuLaneDistance + delta);
		}
	}
	TrajectoryPlanner::TrajectoryPlanningInJunction(simuVehicle, simuInterDistance, simuSpeed, simuLocation, simuAngle, simuOmega, plan);

}


void TrajectoryPlanner::TrajectoryPlanningCaseC(DrivingPlan& plan, Vehicle simuVehicle, Vehicle mappingVehicle)
{
	int index = 0;

	double simuLaneDistance = simuVehicle.LeftLaneDistance;
	double simuInterDistance = simuVehicle.LeftInterDistance;
	double delta = simuInterDistance;
	double mapDistance = mappingVehicle.LeftDistance;

	BPointCoordinate simuLocation = *new BPointCoordinate();
	simuLocation.X = simuVehicle.Location.X;
	simuLocation.Y = simuVehicle.Location.Y;

	double simuAngle = simuVehicle.PoseAngle;
	double simuSpeed = simuVehicle.Speed;
	double simuOmega = 0;

	list<double> mappSpeedList;

	list<double> mappDistList = *new list<double>();

	if (mappingVehicle.PlanState ==VehiclePlanState::Allocated)
	{
		mappSpeedList = TrajectoryPlanner::_drivingPlanDict[mappingVehicle.Id].SpeedList;
		mappDistList = TrajectoryPlanner::_drivingPlanDict[mappingVehicle.Id].DistanceList;
	}
	else
	{
		mappSpeedList = TrajectoryPlanner::PlanBasedApproach_DrivingPlanDict[mappingVehicle.Id].SpeedList;
		mappDistList = TrajectoryPlanner::PlanBasedApproach_DrivingPlanDict[mappingVehicle.Id].DistanceList;
	}

	double someWhatDistance = TrajectoryPlanner::GetMappingDistance(mappingVehicle, simuVehicle);

	list<double>::iterator mapDistList_index = mappDistList.begin();
	mapDistance = *mapDistList_index;

	list<double>::iterator mapSpeedList_index = mappSpeedList.begin();

	if (simuVehicle.SteeringType != 0)
	{
		while (mapDistance > 0)
		{
			if (simuLaneDistance < 0)
			{
				break;
			}

			double leadDist = simuLaneDistance - mapDistance - someWhatDistance;

			double leadSpeed = *mapSpeedList_index;
			mapSpeedList_index++;

			double upperBoundSpeed = simuSpeed;

			TrajectoryPlanner::MyFollowingBehaviors.PlanBasedFollowingBehaviorWhenMapping(mappingVehicle.NoiseInDistance, leadDist, leadSpeed, upperBoundSpeed, simuVehicle, simuLaneDistance);

			if ((simuSpeed * simuSpeed - simuVehicle.MaxSteerSpeed * simuVehicle.MaxSteerSpeed) / 2 / simuVehicle.MaxStraightAccel > simuLaneDistance)
			{
				simuSpeed = fmax(simuVehicle.MaxSteerSpeed, simuSpeed - simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}
			else
			{
				simuSpeed = fmin(upperBoundSpeed, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;

			mapDistList_index++;
			index++;
			mapDistance = *mapDistList_index;
			

			plan.DistanceList.push_back(simuLaneDistance + delta);
		}

		while (simuLaneDistance > 0)
		{
			if ((simuSpeed * simuSpeed - simuVehicle.MaxSteerSpeed * simuVehicle.MaxSteerSpeed) / 2 / simuVehicle.MaxStraightAccel >
				simuLaneDistance)
			{
				simuSpeed = fmax(simuVehicle.MaxSteerSpeed,
					simuSpeed - simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}
			else
			{
				simuSpeed = fmin(simuVehicle.MaxStraightSpeed,
					simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			plan.DistanceList.push_back(simuLaneDistance + delta);
		}
	}
	else 
	{
		while (mapDistance > 0)
		{
			if (simuLaneDistance < 0)
			{
				break;
			}
			double mappDist = simuLaneDistance - mapDistance - someWhatDistance;
			double mappSpeed = *mapSpeedList_index;
			mapSpeedList_index++;

			double upperBoundSpeed = simuSpeed;

			TrajectoryPlanner::MyFollowingBehaviors.PlanBasedFollowingBehaviorWhenMapping(mappingVehicle.NoiseInDistance, mappDist, mappSpeed, upperBoundSpeed,simuVehicle, simuLaneDistance);

			simuSpeed = fmin(upperBoundSpeed, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			
			mapDistList_index++;
			index++;
			mapDistance = *mapDistList_index;

			plan.DistanceList.push_back(simuLaneDistance + delta);
		}
		while (simuLaneDistance > 0)
		{
			simuSpeed = fmin(simuVehicle.MaxStraightSpeed,
				simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;

			plan.DistanceList.push_back(simuLaneDistance + delta);
		}

	}
	TrajectoryPlanningInJunction(simuVehicle, simuInterDistance, simuSpeed, simuLocation, simuAngle, simuOmega, plan);

}

void TrajectoryPlanner::TrajectoryPlanningCaseD(DrivingPlan& plan, Vehicle simuVehicle, Vehicle leadingVehicle, Vehicle mappingVehicle)
{
	int index = 0;

	double simuLaneDistance = simuVehicle.LeftLaneDistance;
	double simuInterDistance = simuVehicle.LeftInterDistance;
	double mappDistance = mappingVehicle.LeftDistance;
	double leadLaneDistance = leadingVehicle.LeftLaneDistance;
	double delta = simuInterDistance;

	BPointCoordinate simuLocation = *new BPointCoordinate();
	simuLocation.X = simuVehicle.Location.X;
	simuLocation.Y = simuVehicle.Location.Y;


	double simuAngle = simuVehicle.PoseAngle;
	double simuSpeed = simuVehicle.Speed;
	double simuOmega = 0;

	list<double> leadSpeedList;
	if (leadingVehicle.PlanState == VehiclePlanState::Allocated)
	{
		leadSpeedList = TrajectoryPlanner::_drivingPlanDict[leadingVehicle.Id].SpeedList;
	}
	else
	{
		leadSpeedList = TrajectoryPlanner::PlanBasedApproach_DrivingPlanDict[leadingVehicle.Id].SpeedList;
	}
	list<double>::iterator leadSpeedList_index = leadSpeedList.begin();

	list<double> mappSpeedList;
	list<double> mappDistList = *new list<double>();
	if (mappingVehicle.PlanState == VehiclePlanState::Allocated)
	{
		mappSpeedList = TrajectoryPlanner::_drivingPlanDict[mappingVehicle.Id].SpeedList;
		mappDistList = TrajectoryPlanner::_drivingPlanDict[mappingVehicle.Id].DistanceList;
	}
	else
	{
		mappSpeedList = TrajectoryPlanner::PlanBasedApproach_DrivingPlanDict[mappingVehicle.Id].SpeedList;
		mappDistList = TrajectoryPlanner::PlanBasedApproach_DrivingPlanDict[mappingVehicle.Id].DistanceList;
	}

	list<double>::iterator mappDistList_index = mappDistList.begin();
	mappDistance = *mappDistList_index;

	list<double>::iterator mappSpeedList_index = mappSpeedList.begin();

	double someWhatDistance = GetMappingDistance(mappingVehicle, simuVehicle);

	if (simuVehicle.SteeringType != 0)
	{
		double leadUpBound;
		double mappUpBound;

		while (simuLaneDistance > 0)
		{
			leadUpBound = simuVehicle.MaxStraightSpeed;
			mappUpBound = simuVehicle.MaxStraightSpeed;

			if (leadLaneDistance > 0)
			{
				double leadDist = simuLaneDistance - leadLaneDistance;
				plan.LeadDistList.push_back(leadDist);
				double leadSpeed = *leadSpeedList_index;
				leadSpeedList_index++;

				double upperBoundSpeed = simuSpeed;

				TrajectoryPlanner::MyFollowingBehaviors.AgentBasedFollowingBehavior(leadingVehicle.NoiseInDistance, leadDist, leadSpeed, upperBoundSpeed,simuVehicle);

				leadUpBound = upperBoundSpeed;
				leadLaneDistance -= TrajectoryPlanner::DeltaTime * leadSpeed;
			}


			if (mappDistance > 0)
			{
				double leadDist = simuLaneDistance - mappDistance - someWhatDistance;
				double mappSpeed = *mappSpeedList_index;
				mappSpeedList_index++;

				double upperBoundSpeed = simuSpeed;
				TrajectoryPlanner::MyFollowingBehaviors.PlanBasedFollowingBehaviorWhenMapping(mappingVehicle.NoiseInDistance, leadDist, mappSpeed,upperBoundSpeed,simuVehicle, simuLaneDistance);

				mappUpBound = upperBoundSpeed;
				mappDistList_index++;
				mappDistance = *mappDistList_index;
			}


			double totalUpBound = fmin(leadUpBound, mappUpBound);

			if ((simuSpeed * simuSpeed - simuVehicle.MaxSteerSpeed * simuVehicle.MaxSteerSpeed) / 2 / simuVehicle.MaxStraightAccel >
				simuLaneDistance)
			{
				simuSpeed = fmax(simuVehicle.MaxSteerSpeed,
					simuSpeed - simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}
			else
			{
				simuSpeed = fmin(simuVehicle.MaxStraightSpeed,
					simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			}
			simuSpeed = fmin(totalUpBound, simuSpeed);

			index++;

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			plan.DistanceList.push_back(simuLaneDistance + delta);
		}
	}
	else
	{
		double leadUpBound;
		double mappUpBound;

		while (simuLaneDistance > 0)
		{
			leadUpBound = simuVehicle.MaxStraightSpeed;
			mappUpBound = simuVehicle.MaxStraightSpeed;

			if (leadLaneDistance > 0)
			{
				double leadDist = simuLaneDistance - leadLaneDistance;
				plan.LeadDistList.push_back(leadDist);
				double leadSpeed = *leadSpeedList_index;
				double upperBoundSpeed = simuSpeed;
				TrajectoryPlanner::MyFollowingBehaviors.AgentBasedFollowingBehavior(leadingVehicle.NoiseInDistance, leadDist, leadSpeed, upperBoundSpeed,simuVehicle);
				leadUpBound = upperBoundSpeed;
				leadLaneDistance -=  TrajectoryPlanner::DeltaTime * leadSpeed;
			}


			if (mappDistance > 0)
			{
				double leadDist = simuLaneDistance - mappDistance - someWhatDistance;
				double mappSpeed = *mappSpeedList_index;
				double upperBoundSpeed = simuSpeed;
				TrajectoryPlanner::MyFollowingBehaviors.PlanBasedFollowingBehaviorWhenMapping(mappingVehicle.NoiseInDistance, leadDist, mappSpeed,upperBoundSpeed,simuVehicle, simuLaneDistance);

				mappUpBound = upperBoundSpeed;

				mappDistList_index++;
				mappDistance = *mappDistList_index;
			}
			double totalUpBound = fmin(leadUpBound, mappUpBound);

			simuSpeed = fmin(totalUpBound, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);

			index++;

			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuLaneDistance -=TrajectoryPlanner::DeltaTime * simuSpeed;

			plan.DistanceList.push_back(simuLaneDistance + delta);
		}
	}
	TrajectoryPlanner::TrajectoryPlanningInJunction(simuVehicle, simuInterDistance, simuSpeed, simuLocation, simuAngle, simuOmega, plan);

}

double TrajectoryPlanner::GetMappingDistance(Vehicle formerVehicle, Vehicle latterVehicle)
{
	if (formerVehicle.SteeringType == -1)
	{
		if ((latterVehicle.FromNodeId - formerVehicle.FromNodeId + 4) % 4 == 2)
		{
			return 3;
		}
	}

	if (formerVehicle.SteeringType == 0)
	{
		if ((latterVehicle.FromNodeId - formerVehicle.FromNodeId + 4) % 4 == 3)
		{
			return 3;
		}
	}
	return 0;
}

double TrajectoryPlanner::GetOmegaForTurningLeft(double radius, double simuSpeed, double& calAngle)
{
	double simuOmega;
	double theta = 2 * asin(simuSpeed * TrajectoryPlanner::DeltaTime / 2 / radius);
	if (calAngle + theta > acos(-1) / 2)
	{
		simuOmega = (acos(-1) / 2 - calAngle) / TrajectoryPlanner::DeltaTime;
	}
	else
	{
		simuOmega = theta / TrajectoryPlanner::DeltaTime;
	}
	calAngle += simuOmega * TrajectoryPlanner::DeltaTime;
	return simuOmega;
}

double TrajectoryPlanner::GetOmegaForTurningRight(double radius, double simuSpeed, double& calAngle)
{
	double simuOmega;
	double theta = 2 * asin(simuSpeed * TrajectoryPlanner::DeltaTime / 2 / radius);
	if (calAngle + theta > acos(-1) / 2)
	{
		simuOmega = -(acos(-1) / 2 - calAngle) / TrajectoryPlanner::DeltaTime;
	}
	else
	{
		simuOmega = -theta / TrajectoryPlanner::DeltaTime;
	}
	calAngle += -simuOmega * TrajectoryPlanner::DeltaTime;
	return simuOmega;
}

void TrajectoryPlanner::TrajectoryPlanningInMergeZone(Vehicle simuVehicle, double simuInterDistance, BPointCoordinate simuLocation, double simuAngle, double simuOmega, DrivingPlan& plan)
{
	double calAngle = 0;
	double epsilon = 0.000001;
	double simuSpeed = simuVehicle.MaxSteerSpeed;

	switch (simuVehicle.SteeringType)
	{
	case 0:
		while (simuInterDistance > 0)
		{
			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, 0);
			simuInterDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			if (simuInterDistance < epsilon)
			{
				simuInterDistance = 0;
			}
			plan.DistanceList.push_back(simuInterDistance);
		}
		break;
	case -1:
		while (simuInterDistance > 0)
		{
			simuOmega = TrajectoryPlanner::GetOmegaForTurningLeft(simuVehicle.SteeringRadius, simuSpeed, calAngle);

			simuAngle += simuOmega * TrajectoryPlanner::DeltaTime;
			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuInterDistance -= TrajectoryPlanner::DeltaTime * simuOmega * simuVehicle.SteeringRadius;
			if (simuInterDistance < epsilon)
			{
				simuInterDistance = 0;
			}
			plan.DistanceList.push_back(simuInterDistance);
		}
		break;
	case 1:
		while (simuInterDistance > 0)
		{
			simuOmega = GetOmegaForTurningRight(simuVehicle.SteeringRadius, simuSpeed, calAngle);

			simuAngle += simuOmega * TrajectoryPlanner::DeltaTime;
			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuInterDistance += TrajectoryPlanner::DeltaTime * simuOmega * simuVehicle.SteeringRadius;
			if (simuInterDistance < epsilon)
			{
				simuInterDistance = 0;
			}
			plan.DistanceList.push_back(simuInterDistance);

		}
		break;
	default:
		break;
	}
}

void TrajectoryPlanner::TrajectoryPlanningInJunction(Vehicle simuVehicle, double simuInterDistance, double simuSpeed, BPointCoordinate simuLocation, double simuAngle, double simuOmega, DrivingPlan& plan)
{
	double calAngle = 0;
	double epsilon = 0.000001;
	switch (simuVehicle.SteeringType)
	{
	case 0:
		while (simuInterDistance > 0)
		{
			simuSpeed = fmin(simuVehicle.MaxStraightSpeed, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, 0);
			simuInterDistance -= TrajectoryPlanner::DeltaTime * simuSpeed;
			if (simuInterDistance < epsilon)
			{
				simuInterDistance = 0;
			}
			plan.DistanceList.push_back(simuInterDistance);
		}
		break;
	case -1:
		while (simuInterDistance>0)
		{
			simuSpeed = fmin(simuVehicle.MaxSteerSpeed, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			simuOmega = TrajectoryPlanner::GetOmegaForTurningLeft(simuVehicle.SteeringRadius, simuSpeed, calAngle);

			simuAngle += simuOmega * TrajectoryPlanner::DeltaTime;
			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuInterDistance -= TrajectoryPlanner::DeltaTime * simuOmega * simuVehicle.SteeringRadius;
			if (simuInterDistance < epsilon)
			{
				simuInterDistance = 0;
			}
			plan.DistanceList.push_back(simuInterDistance);
		}
		break;
	case 1:
		while (simuInterDistance > 0)
		{
			simuSpeed = fmin(simuVehicle.MaxSteerSpeed, simuSpeed + simuVehicle.MaxStraightAccel * TrajectoryPlanner::DeltaTime);
			simuOmega = GetOmegaForTurningRight(simuVehicle.SteeringRadius, simuSpeed, calAngle);

			simuAngle += simuOmega * TrajectoryPlanner::DeltaTime;
			simuLocation.X += simuSpeed * cos(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			simuLocation.Y += simuSpeed * sin(simuAngle + simuVehicle.Direction) * TrajectoryPlanner::DeltaTime;
			plan.Add(simuLocation, simuAngle, simuSpeed, simuOmega);

			simuInterDistance += TrajectoryPlanner::DeltaTime * simuOmega * simuVehicle.SteeringRadius;
			if (simuInterDistance < epsilon)
			{
				simuInterDistance = 0;
			}
			plan.DistanceList.push_back(simuInterDistance);

		}
		break;
	default:
		break;
	}
}

double TrajectoryPlanner::CalculateEstTime(Vehicle vehicle, Vehicle leadingVehicle, Vehicle mappingVehicle, int typeIndex)
{
	double estT = TrajectoryPlanner::EstimateTime(vehicle);
	double leadEstT;
	switch (typeIndex)
	{
	case 1:
		return estT;

	case 2:
		leadEstT = LeadEstTime(vehicle, leadingVehicle);
		return fmax(estT, leadEstT);

	case 3:
		return fmax(estT, mappingVehicle.TotalEst);

	case 4:
		leadEstT = LeadEstTime(vehicle, leadingVehicle);
		return fmax(estT, fmax(leadEstT, mappingVehicle.TotalEst));
	}

	return 0;
}

double TrajectoryPlanner::LeadEstTime(Vehicle vehicle, Vehicle leadingVehicle)
{
	if(vehicle.SteeringType==0)
	{
		return leadingVehicle.TotalEst - leadingVehicle.InterTime + TrajectoryPlanner::MaintainingDistance / vehicle.MaxStraightSpeed;
	}
	else
	{
		return leadingVehicle.TotalEst - leadingVehicle.InterTime +
			(TrajectoryPlanner::MaintainingDistance -
				(vehicle.MaxStraightSpeed * vehicle.MaxStraightSpeed -
					vehicle.MaxSteerSpeed * vehicle.MaxSteerSpeed) / 2 /
				vehicle.MaxStraightAccel) / vehicle.MaxStraightSpeed +
			(vehicle.MaxStraightSpeed - vehicle.MaxSteerSpeed) /
			vehicle.MaxStraightAccel;
	}
}

double TrajectoryPlanner::EstimateTime(Vehicle vehicle)
{
	double V0 = vehicle.Speed;
	double V = vehicle.MaxStraightSpeed;
	double a = vehicle.MaxStraightAccel;
	double Vs = vehicle.MaxSteerSpeed;
	double L = vehicle.LeftLaneDistance;

	if (vehicle.SteeringType == 0)
	{
		return (V - V0) / a + (L - (V * V - V0 * V0) / 2 / a) / V;
	}

	else
	{
		if (L > (V * V - V0 * V0) / 2 / a + (V * V - Vs * Vs) / 2 / a)
		{
			return (V - V0) / a + (L - (2 * V * V - V0 * V0 - Vs * Vs) / 2 / a) / V + (V - Vs) / a;
		}
		else
		{
			return (sqrt(4 * a * L + 2 * Vs * Vs + 2 * V0 * V0) - V0 - Vs) / a;
		}
	}
}

double TrajectoryPlanner::CalculateInterTime(Vehicle vehicle)
{
	if (vehicle.SteeringType == 0)
	{
		return vehicle.LeftInterDistance / vehicle.MaxStraightSpeed;
	}
	else
	{
		return vehicle.LeftInterDistance / vehicle.MaxSteerSpeed;
	}
}

bool TrajectoryPlanner::Debug_Is_Corrected(Vehicle debugVehicle)
{
	return 1;
}

void TrajectoryPlanner::UploadPlanningResult(map<int, DrivingPlan>& aPlanBasedApproach_DrivingPlanDict, map<int, Vehicle>& aSimuVehicleDict)
{
	for (map<int, DrivingPlan>::iterator DPIter = TrajectoryPlanner::_drivingPlanDict.begin(); DPIter != TrajectoryPlanner::_drivingPlanDict.end(); DPIter++)
	{
		aPlanBasedApproach_DrivingPlanDict[DPIter->first] = DPIter->second;
	}

	
	for (map<int, Vehicle>::iterator VIter = TrajectoryPlanner::SimuVehicleDict.begin(); VIter != TrajectoryPlanner::SimuVehicleDict.end(); VIter++)
	{
		aSimuVehicleDict[VIter->first] = VIter->second;
	}
}
