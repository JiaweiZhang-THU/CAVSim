#include "SimulatorMerging.h"
#include "SimulatorMergingParameters.h"

SimulatorMerging::SimulatorMerging()
{
	SimulatorMerging::SimulatingStep = 0;
	SimulatorMerging::SimulatingStep2Time = 0;
	SimulatorMerging::TimeStep = paraTimeStepMerging;
	SimulatorMerging::TimeIntervalForUpdateTrafficFlowRate = paraTimeIntervalForUpdateTrafficFlowRateMerging;

	SimulatorMerging::LaneNum = paraDefaultLaneNumDefinedSLBPMerging;

	SimulatorMerging::RadiusOfRampControlZone = paraRadiusOfRampControlZone;

	SimulatorMerging::VehileFlowPossionLambda = paraVehileFlowPossionLambdaPerLaneMerging;

	SimulatorMerging::AccumulaterdVehicleNum = 0;

	SimulatorMerging::TimeStepIntervalBetweenSnapshot = 10;

	SimulatorMerging::ThresholdOfVehicleNumInSignalLanUnit = (int)50 / 5;

	SimulatorMerging::SimulatorId = int(rand() % 1000 + 8000);

	SimulatorMerging::IsLoadHLCMModel = false;

	SimulatorMerging::IsForHLCMTest = false;

	SimulatorMerging::IsNoLaneChangeForComparision = false;
}

list<int> SimulatorMerging::VehicleFlowGenerator(int aVehicleNum, int aBeginTime)
{
	list<double> TimeInterval;
	list<double>::iterator localDoubleIter;
	for (int i = 0; i < aVehicleNum; i++)
	{
		TimeInterval.push_back(SimulatorMerging::RandomExponential());
	}

	list<int> VehicleFlow;
	double localIntervalSum = 0;
	for (localDoubleIter = TimeInterval.begin(); localDoubleIter != TimeInterval.end(); localDoubleIter++)
	{
		localIntervalSum += *localDoubleIter;
		VehicleFlow.push_back(int(localIntervalSum + aBeginTime));
	}

	return VehicleFlow;
}


list<int> SimulatorMerging::VehicleFlowReadFromFile()
{
	list<int> VehicleFlow;
	return VehicleFlow;
}

BNode SimulatorMerging::InitialBNodeForNewVehicle()
{
	list<BNode> localEntranceOnRoadNetwork = SimulatorMerging::MergingScenario.EnteringVehicleInitialPositonAndPoseList;
	int EntranceNum = int(localEntranceOnRoadNetwork.size());
	int RandomEntrance = SimulatorMerging::AccumulaterdVehicleNum % EntranceNum; 

	list<BNode>::iterator localIter = localEntranceOnRoadNetwork.begin();
	for (int i = 0; i < RandomEntrance; i++)
	{
		localIter++;
	}
	return *localIter;
}

double SimulatorMerging::RandomExponential()
{
	double ExponentialValue = 0.0;
	while (true)
	{
		ExponentialValue = (double)rand() / (double)RAND_MAX;
		if (ExponentialValue != 1)
		{
			break;
		}
	}
	ExponentialValue = (-1.0 / SimulatorMerging::VehileFlowPossionLambda) * log(1 - ExponentialValue);
	return ExponentialValue;
}

bool SimulatorMerging::CreateNewVehicle()
{
	return true;
}

void SimulatorMerging::ClearVehicle()
{
	SimulatorMerging::VehicleList.clear();
}

void SimulatorMerging::UpdateVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = SimulatorMerging::VehicleList.begin(); localVehicleIterator != SimulatorMerging::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			*localVehicleIterator = aVehicle;
		}
	}
}

void SimulatorMerging::AddVehicle(Vehicle aVehicle)
{
	SimulatorMerging::VehicleList.push_back(aVehicle);
}

void SimulatorMerging::RemoveVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = SimulatorMerging::VehicleList.begin(); localVehicleIterator != SimulatorMerging::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			SimulatorMerging::VehicleList.erase(localVehicleIterator);
			break;
		}
	}
}

void SimulatorMerging::UpdateVehicle2MapBlock(Vehicle& aVehicle)
{
	if (aVehicle.TimeOfEnteringIntersectionCircleControlZone == -1 && aVehicle.LeftLaneDistance < SimulatorMerging::RadiusOfRampControlZone)
	{
		aVehicle.TimeOfEnteringIntersectionCircleControlZone = SimulatorMerging::SimulatingStep;
	}

	if (aVehicle.TimeOfEnteringIntersectionConflictZone == -1 && aVehicle.LeftLaneDistance < 0.5)
	{
		aVehicle.TimeOfEnteringIntersectionConflictZone = SimulatorMerging::SimulatingStep;
	}
	if (aVehicle.LaneUnitPointer->IsInside(aVehicle) == true)
	{
		aVehicle.LaneUnitPointer->UpdateVehicle(aVehicle);
	}
	else
	{
		LaneUnit* localLaneUnitPointer = NULL;
		bool localFlagIsAdjacentUnitFound = false;

		for (int i = 0; i < 9; i++)
		{
			localLaneUnitPointer = aVehicle.LaneUnitPointer->AdjacentUnitPointer[i];
			if (localLaneUnitPointer == NULL)
			{
				continue;
			}
			if (localLaneUnitPointer->IsInside(aVehicle) == true)
			{
				localFlagIsAdjacentUnitFound = true;
				localLaneUnitPointer->AddVehicle(aVehicle);
				aVehicle.LaneUnitPointer->RemoveVehicle(aVehicle);
				aVehicle.LaneUnitPointer = localLaneUnitPointer;
				aVehicle.RoadId = SimulatorMerging::MergingScenario.Id;
				int localPreviousBlockId = aVehicle.BlockId;
				aVehicle.BlockId = aVehicle.LaneUnitPointer->BlockId;
				if (localPreviousBlockId == 4)
				{
					aVehicle.Direction = 0.0;
					aVehicle.PoseAngle = 0.0;
					aVehicle.Location.Y = 2.0;
				}
				aVehicle.LaneId = aVehicle.LaneUnitPointer->Id;
				break;
			}
		}
		if (localFlagIsAdjacentUnitFound == false)
		{
			aVehicle.LaneUnitPointer->RemoveVehicle(aVehicle);
			SimulatorMerging::VehicleOverList.push_back(aVehicle);
		}

	}
	if (aVehicle.BlockId == 3)
	{
		aVehicle.MaxStraightSpeed = 16;
	}
}

void SimulatorMerging::CheckWhetherSimulationNeedToBeReset()
{
	bool localIsNeedToResetEnv = false;
	for (list<LaneUnit>::iterator localLaneUnitPointer = SimulatorMerging::MergingScenario.Zone1_EnterMainRoad.StraightMultiLanes.begin(); localLaneUnitPointer != SimulatorMerging::MergingScenario.Zone1_EnterMainRoad.StraightMultiLanes.end(); localLaneUnitPointer++)
	{
		if (localLaneUnitPointer->VehicleList.size() > SimulatorMerging::ThresholdOfVehicleNumInSignalLanUnit)
		{
			localIsNeedToResetEnv = true;
			break;
		}
	}
	if (localIsNeedToResetEnv == true)
	{
		SimulatorMerging::VehicleOverList.clear();
		for (list<Vehicle>::iterator localVehicleIter = SimulatorMerging::VehicleList.begin(); localVehicleIter != SimulatorMerging::VehicleList.end(); localVehicleIter++)
		{
			localVehicleIter->LaneUnitPointer->RemoveVehicle(*localVehicleIter);
			SimulatorMerging::VehicleOverList.push_back(*localVehicleIter);
		}
	}
}


bool SimulatorMerging::IsLaneChangingPermitted(Vehicle aVehicel, int aLaneChangeAction)
{
	if (aVehicel.LaneId > 1 && aVehicel.LaneId < SimulatorMerging::LaneNum)
	{
		return true;
	}
	else
	{
		if (aVehicel.LaneId == 1 && aLaneChangeAction == -1)
		{
			return false;
		}
		else
		{
			if (aVehicel.LaneId == SimulatorMerging::LaneNum && aLaneChangeAction == 1)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
	}
	return false;
}

list<Vehicle> SimulatorMerging::PerceptionSurroundingVehicles(Vehicle aVehicle)
{
	list<Vehicle> SurroundingVehicleList;
	LaneUnit* localLaneUnitPointer = NULL;
	list<Vehicle>::iterator localVehicleIterator;

	for (int i = 0; i < 9; i++)
	{
		localLaneUnitPointer = aVehicle.LaneUnitPointer->AdjacentUnitPointer[i];
		if (localLaneUnitPointer == NULL)
		{
			continue;
		}
		for (localVehicleIterator = localLaneUnitPointer->VehicleList.begin(); localVehicleIterator != localLaneUnitPointer->VehicleList.end(); localVehicleIterator++)
		{
			if (localVehicleIterator->Id == aVehicle.Id)
			{
				continue;
			}
			else
			{
				SurroundingVehicleList.push_back(*localVehicleIterator);
			}

		}

	}
	return SurroundingVehicleList;
}


list<Vehicle> SimulatorMerging::PerceptionSurroundingVehiclesPlus(Vehicle aVehicle)
{
	list<Vehicle> SurroundingVehicleList;
	LaneUnit* localLaneUnitPointer = NULL;
	list<Vehicle>::iterator localVehicleIterator;

	for (int i = 0; i < 9; i++)
	{
		localLaneUnitPointer = aVehicle.LaneUnitPointer->AdjacentUnitPointer[i];
		if (localLaneUnitPointer == NULL)
		{
			continue;
		}
		for (localVehicleIterator = localLaneUnitPointer->VehicleList.begin(); localVehicleIterator != localLaneUnitPointer->VehicleList.end(); localVehicleIterator++)
		{
			if (localVehicleIterator->Id == aVehicle.Id)
			{
				continue;
			}
			else
			{
				SurroundingVehicleList.push_back(*localVehicleIterator);
			}

		}

	}
	return SurroundingVehicleList;
}

bool SimulatorMerging::CooperativeDrivingAtOnRamp()
{
	SimulatorMerging::MergingScenario.ConflictBoundaryPointOnMainRoad.X = SimulatorMerging::MergingScenario.Zone1_EnterMainRoad.To_Node.Location.X;
	SimulatorMerging::MergingScenario.ConflictBoundaryPointOnMainRoad.Y = SimulatorMerging::MergingScenario.Zone1_EnterMainRoad.To_Node.Location.Y + paraLaneWidthInSimulatorMerging / 2;
	SimulatorMerging::MergingScenario.ConflictBoundaryPointOnRamp.X = SimulatorMerging::MergingScenario.Zone4_MergingLane.To_Node.Location.X - paraLaneWidthInSimulatorMerging / 2 * sin(SimulatorMerging::MergingScenario.AngleOfMergingLane);
	SimulatorMerging::MergingScenario.ConflictBoundaryPointOnRamp.Y = SimulatorMerging::MergingScenario.Zone4_MergingLane.To_Node.Location.Y + paraLaneWidthInSimulatorMerging / 2 * cos(SimulatorMerging::MergingScenario.AngleOfMergingLane);

	SimulatorMerging::MyPlanBasedApproach = new PlanBasedApproach_ramp(SimulatorMerging::MergingScenario.NumOfLanesOnMainRoad, SimulatorMerging::MergingScenario.ConflictBoundaryPointOnMainRoad, SimulatorMerging::MergingScenario.ConflictBoundaryPointOnRamp, SimulatorMerging::SimulatingStep);
	SimulatorMerging::MyPlanBasedApproach->Init();
	SimulatorMerging::MyPlanBasedApproach->DrivingInVehiclesListArray->clear();

	list<LaneUnit>::iterator localLUIter;
	for (int BlockId = 1; BlockId < 5; BlockId++) {
		int isInRamp = 0;
		if (BlockId == 1) {
			localLUIter = SimulatorMerging::MergingScenario.Zone1_EnterMainRoad.StraightMultiLanes.begin();
		}
		else if (BlockId == 4) {
			localLUIter = SimulatorMerging::MergingScenario.Zone4_MergingLane.StraightMultiLanes.begin();
			isInRamp = 1;
		}
		else {
			continue;
		}

		if (localLUIter->VehicleList.size() == 0) {
			continue;
		}
		for (list<Vehicle>::iterator VIter = localLUIter->VehicleList.begin(); VIter != localLUIter->VehicleList.end(); VIter++) {
			SimulatorMerging::MyPlanBasedApproach->DrivingInVehiclesListArray[isInRamp].push_back(*VIter);
		}
	}

	bool HasNewLockedVehicle = false;
	bool localInfo = SimulatorMerging::MyPlanBasedApproach->Run(SimulatorMerging::VehicleDrivingPlan, HasNewLockedVehicle);
	if (localInfo == true) {
		SimulatorMerging::MyPlanBasedApproach->UpdatePlanningResults(SimulatorMerging::VehicleList, SimulatorMerging::VehicleDrivingPlan);
	}
	if (HasNewLockedVehicle == true) {
		SimulatorMerging::MyPlanBasedApproach->UpdateRampArrivalTime(SimulatorMerging::VehicleList);
	}

	return localInfo;
}

void SimulatorMerging::Run()
{
	SimulatorMerging::VehileFlowPossionLambda = 300.0 / (3600 * 10);
	 list<int> VehicleArrivingTimetable = SimulatorMerging::VehicleFlowGenerator(5000, 0);
	 list<int>::iterator localVehicleArrivingTimePoint = VehicleArrivingTimetable.begin();

	SimulatorMerging::MyTrajectoryPlanning.ChoosedCarFollowingModel = 1;
	SimulatorMerging::MyTrajectoryPlanning.IsNeedCollisionCheck = true;
	double localEfficientAndSafetyRewardWeight3 = 4;
	SimulatorMerging::SimulatingStep = 0;
	SimulatorMerging::SimulatingStep2Time = 0;

	ofstream localWritingIn;
	localWritingIn.open("Data/OnRamp_" + to_string(SimulatorMerging::SimulatorId) + ".txt", ios::out);

	clock_t localStartTime, localEndTime;
	localStartTime = clock();

	int localDecisonNums = 0;
	int localSchedualTimeInterval = 20; 

	while (true)
	{
		SimulatorMerging::SimulatingStep += 1;
		SimulatorMerging::SimulatingStep2Time += SimulatorMerging::TimeStep;

		if (SimulatorMerging::SimulatingStep % 20 == 0)
		{
			cout << "--------------------- Simulation ID:" << SimulatorMerging::SimulatorId << " --- RunStep: " << SimulatorMerging::SimulatingStep << " --- Time: " << SimulatorMerging::SimulatingStep2Time << " s --------------------------------" << endl;
		}
		if (SimulatorMerging::SimulatingStep > *localVehicleArrivingTimePoint)
		{
			SimulatorMerging::AccumulaterdVehicleNum += 1;

			if (*localVehicleArrivingTimePoint == VehicleArrivingTimetable.back())
			{
				VehicleArrivingTimetable = SimulatorMerging::VehicleFlowGenerator(paraPerFlowVehicleNumMerging, SimulatorMerging::SimulatingStep);
				localVehicleArrivingTimePoint = VehicleArrivingTimetable.begin();
			}
			else
			{
				localVehicleArrivingTimePoint++;
			}
			BNode InitialBNodeForNewVehicle = SimulatorMerging::InitialBNodeForNewVehicle();
			Vehicle NewVehicle(InitialBNodeForNewVehicle.Location, 0);
			if (InitialBNodeForNewVehicle.Id == SimulatorMerging::MergingScenario.NumOfLanesOnMainRoad + 1)
			{
				NewVehicle.WhetherTheInitialPositionIsOnTheMainRoad = false;
			}
			else
			{
				NewVehicle.WhetherTheInitialPositionIsOnTheMainRoad = true;
			}
			NewVehicle.Direction = InitialBNodeForNewVehicle.Direction;
			NewVehicle.Id = SimulatorMerging::AccumulaterdVehicleNum;
			NewVehicle.RoadId = SimulatorMerging::MergingScenario.Id;
			if (InitialBNodeForNewVehicle.Id == 1 or InitialBNodeForNewVehicle.Id == 2)
			{
				NewVehicle.BlockId = 1;
				NewVehicle.LaneId = InitialBNodeForNewVehicle.Id;
			}
			else
			{
				NewVehicle.BlockId = 4;
				NewVehicle.LaneId = 1;
			}
			NewVehicle.LaneUnitPointer = SimulatorMerging::MergingScenario.FindLaneUnitPointer(NewVehicle.BlockId, NewVehicle.LaneId);
			NewVehicle.Speed = NewVehicle.MaxStraightSpeed;
			NewVehicle.BornTime = SimulatorMerging::SimulatingStep;
			NewVehicle.LaneUnitPointer->AddVehicle(NewVehicle);
			NewVehicle.LeftLaneDistance = SimulatorMerging::MergingScenario.EnterMainRoadLength;
			SimulatorMerging::VehicleList.push_back(NewVehicle);
		}

		list<Vehicle> localPerceptionSurroundingVehiclesList;
		list<Vehicle> localPerceptionSurroundingVehiclesListPlus;
		list<Vehicle>::iterator localVehichIterator;

		if ((SimulatorMerging::SimulatingStep + 1) % localSchedualTimeInterval == 0)
		{
			for (localVehichIterator = SimulatorMerging::VehicleList.begin(); localVehichIterator != SimulatorMerging::VehicleList.end(); localVehichIterator++)
			{
				localPerceptionSurroundingVehiclesList = SimulatorMerging::PerceptionSurroundingVehicles(*localVehichIterator);
				localVehichIterator->UpdatePerceptionSurroundingIntersectionVehicles(localPerceptionSurroundingVehiclesList, *localVehichIterator);
				SimulatorMerging::UpdateVehicle2MapBlock(*localVehichIterator);
			}

			bool isPlanning = SimulatorMerging::CooperativeDrivingAtOnRamp();

		}
		for (SimulatorMerging::VehicleOverList.clear(), localVehichIterator = SimulatorMerging::VehicleList.begin(); localVehichIterator != SimulatorMerging::VehicleList.end(); localVehichIterator++)
		{
			localPerceptionSurroundingVehiclesList = SimulatorMerging::PerceptionSurroundingVehicles(*localVehichIterator);
			localPerceptionSurroundingVehiclesListPlus = SimulatorMerging::PerceptionSurroundingVehiclesPlus(*localVehichIterator);
			int localLaneChangeAction = 0;
			if (SimulatorMerging::VehicleDrivingPlan.count(localVehichIterator->Id) == 0 || SimulatorMerging::VehicleDrivingPlan[localVehichIterator->Id].GetCount() == 0)
			{
				SimulatorMerging::MyTrajectoryPlanning.SpeedPlanningOnMergingZone(localVehichIterator->Speed, localVehichIterator->PoseAngle, localPerceptionSurroundingVehiclesList, *localVehichIterator, localLaneChangeAction);
			}
			else
			{
				SimulatorMerging::VehicleDrivingPlan[localVehichIterator->Id].Run(localVehichIterator->Speed, localVehichIterator->AngularSpeed);
				localVehichIterator->PoseAngle += localVehichIterator->AngularSpeed * SimulatorMerging::TimeStep;
			}
			localVehichIterator->UpdateVehicleLocation(SimulatorMerging::TimeStep);
			localVehichIterator->IsCollideWithOtherVehicles(localPerceptionSurroundingVehiclesList);
			SimulatorMerging::UpdateVehicle2MapBlock(*localVehichIterator);
			SimulatorMerging::MyMergingStrategyEvaluation.RunOneStepAtMerging(*localVehichIterator);
			if (1)
			{
				localWritingIn << SimulatorMerging::SimulatingStep
					<< "\t" << localVehichIterator->Id
					<< "\t" << localVehichIterator->Location.X
					<< "\t" << localVehichIterator->Location.Y
					<< "\t" << localVehichIterator->PoseAngle + localVehichIterator->Direction
					<< "\t" << localVehichIterator->Speed
					<< "\n";
			}
		}

		for (localVehichIterator = SimulatorMerging::VehicleOverList.begin(); localVehichIterator != SimulatorMerging::VehicleOverList.end(); localVehichIterator++)
		{
			SimulatorMerging::RemoveVehicle(*localVehichIterator);
		}
		if (SimulatorMerging::SimulatingStep > 25 * 60 * 10)
		{
			break;
		}
	}

	localEndTime = clock();
}
