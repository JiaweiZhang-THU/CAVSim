#include "PlanBasedApproach_ramp.h"
#include "PlanBasedApproachParameters_ramp.h"

#include "stdio.h"
#include "stdlib.h"
#include "time.h"

PlanBasedApproach_ramp::PlanBasedApproach_ramp()
{
	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 20; j++) {
			PlanBasedApproach_ramp::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = 0;
		}
	}
	PlanBasedApproach_ramp::ConflictSubzonesLastPassedVehicleArrivalTime[0][0] = -1;
}

PlanBasedApproach_ramp::PlanBasedApproach_ramp(int aRampMainRoadLanesNum, BPointCoordinate aConflictBoundaryPointOnMainRoad, BPointCoordinate aConflictBoundaryPointOnRamp, double aSimulatorTimeCount)
{
	PlanBasedApproach_ramp::ConsiderRadius = paraControlZoneLength;
	PlanBasedApproach_ramp::LockRadius = paraLockedRadius;
	PlanBasedApproach_ramp::SimulatorTimeDelta = paraDeltaTime_1;

	PlanBasedApproach_ramp::RampMainRoadLanesNum = aRampMainRoadLanesNum;
	PlanBasedApproach_ramp::ConflictBoundaryPointOnMainRoad = aConflictBoundaryPointOnMainRoad;
	PlanBasedApproach_ramp::ConflictBoundaryPointOnRamp = aConflictBoundaryPointOnRamp;
	PlanBasedApproach_ramp::SimulatorTimeCount = aSimulatorTimeCount;

	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 20; j++) {
			PlanBasedApproach_ramp::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = 0;
		}
	}
	PlanBasedApproach_ramp::ConflictSubzonesLastPassedVehicleArrivalTime[0][0] = -1;
}

PlanBasedApproach_ramp::~PlanBasedApproach_ramp()
{
	PlanBasedApproach_ramp::BestSeqList.clear();
	PlanBasedApproach_ramp::RecordList.clear();

	delete PlanBasedApproach_ramp::CurrRecord;
	PlanBasedApproach_ramp::AllSeqList.clear();

	PlanBasedApproach_ramp::SimuVehicleDict.clear();
	PlanBasedApproach_ramp::SimuIdList.clear();

	PlanBasedApproach_ramp::LastLockingList.clear();
	PlanBasedApproach_ramp::ToDeleteList.clear();

	PlanBasedApproach_ramp::DrivingPlanDict.clear();
	PlanBasedApproach_ramp::PlanRecordsList.clear();

	delete PlanBasedApproach_ramp::_scheduleTree_ramp;
	delete PlanBasedApproach_ramp::Planner;
}

void PlanBasedApproach_ramp::Init()
{
	PlanBasedApproach_ramp::ConsiderRadius = paraControlZoneLength;
	PlanBasedApproach_ramp::LockRadius = paraLockedRadius;

	PlanBasedApproach_ramp::BestSeqList = *new list<list<int>>();
	PlanBasedApproach_ramp::RecordList = *new list<TimingRecord>();
	PlanBasedApproach_ramp::CurrRecord = new TimingRecord();
	PlanBasedApproach_ramp::AllSeqList = *new list<list<list<int>>>();
	PlanBasedApproach_ramp::SimuVehicleDict = *new map<int, Vehicle>();
	PlanBasedApproach_ramp::SimuIdList = *new list<int>();

	PlanBasedApproach_ramp::_scheduleTree_ramp = new ScheduleTree_ramp();
	PlanBasedApproach_ramp::_scheduleTree_ramp->Init();

	PlanBasedApproach_ramp::LastLockingList = *new list<Vehicle>();
	PlanBasedApproach_ramp::ToDeleteList = *new list<Vehicle>();
	PlanBasedApproach_ramp::DrivingPlanDict = *new map<int, DrivingPlan>();
	PlanBasedApproach_ramp::PlanRecordsList = *new list<PlanRecord>();

	PlanBasedApproach_ramp::Planner = new TrajectoryPlanner();
	PlanBasedApproach_ramp::Planner->Init();
}

bool PlanBasedApproach_ramp::Run(map<int, DrivingPlan>& aExistVehicleDrivingPlan, bool& HasNewLockedVehicle)
{
	bool localInfo = PlanBasedApproach_ramp::UpdateVehicleRegistrationInfo(aExistVehicleDrivingPlan, HasNewLockedVehicle);

	if (localInfo) {
		PlanBasedApproach_ramp::CurrRecord->UsedTime_Enum = 0;
		PlanBasedApproach_ramp::CurrRecord->UsedTime_Split = 0;
		PlanBasedApproach_ramp::CurrRecord->UsedTime_Plan = 0;
		PlanBasedApproach_ramp::CurrRecord->ConsiderVehicleCount = 0;
		PlanBasedApproach_ramp::CurrRecord->PermutationNum = 0;
		PlanBasedApproach_ramp::CurrRecord->AllPlanCount = 0;

		PlanBasedApproach_ramp::GenerateNewPlan();
		PlanBasedApproach_ramp::AlgorithmInformation = "Spanning tree size: " + PlanBasedApproach_ramp::_scheduleTree_ramp->GetAllPerListCount();

		PlanRecord record = *new PlanRecord();
		list<Vehicle> localVehicleList;
		for (map<int, Vehicle>::iterator mapVIter = PlanBasedApproach_ramp::SimuVehicleDict.begin(); mapVIter != PlanBasedApproach_ramp::SimuVehicleDict.end(); mapVIter++) {
			localVehicleList.push_back(mapVIter->second);
		}
		record.Generate(localVehicleList, PlanBasedApproach_ramp::BestSeqList);
		PlanBasedApproach_ramp::PlanRecordsList.push_back(record);
	}

	return localInfo;
}

bool PlanBasedApproach_ramp::UpdateVehicleRegistrationInfo(map<int, DrivingPlan>& aExistVehicleDrivingPlan, bool& aHasNewLockedVehicle)
{
	bool judgeNewIn = false;
	for (int i = 0; i < 2; i++) {
		list<Vehicle>::iterator localVIterator = PlanBasedApproach_ramp::DrivingInVehiclesListArray[i].begin();
		for (int j = 0; localVIterator != PlanBasedApproach_ramp::DrivingInVehiclesListArray[i].end(); j++, localVIterator++) {
			double localV2IDistance = PlanBasedApproach_ramp::Distance2RampConflictCenter(*localVIterator);
			if (localV2IDistance > PlanBasedApproach_ramp::ConsiderRadius) {
				localVIterator->InIntersectionRadius = VehicleInIntersectionRadius::Outter;
			}
			else {
				if (localV2IDistance < PlanBasedApproach_ramp::LockRadius) {
					localVIterator->InIntersectionRadius = VehicleInIntersectionRadius::Inner;
				}
				else {
					localVIterator->InIntersectionRadius = VehicleInIntersectionRadius::Middle;
				}
			}

			if (localV2IDistance < PlanBasedApproach_ramp::ConsiderRadius) {
				if (localVIterator->PlanState == VehiclePlanState::Free) {
					localVIterator->Arrival = PlanBasedApproach_ramp::SimulatorTimeCount;
					localVIterator->PlanState = VehiclePlanState::Allocated;
					judgeNewIn = true;
					PlanBasedApproach_ramp::SimuVehicleDict[localVIterator->Id] = *localVIterator;
					PlanBasedApproach_ramp::SimuIdList.push_back(localVIterator->Id);

					DrivingPlan plan = *new DrivingPlan();
					PlanBasedApproach_ramp::DrivingPlanDict[localVIterator->Id] = plan;
				}
				else {
					if (localVIterator->PlanState == VehiclePlanState::Allocated) {
						if (localV2IDistance < PlanBasedApproach_ramp::LockRadius) {
							Vehicle inLockSimuVehicle = SimuVehicleDict[localVIterator->Id];
							localVIterator->PlanState = VehiclePlanState::Locaked;
							aHasNewLockedVehicle = true;
							PlanBasedApproach_ramp::SimuVehicleDict.erase(localVIterator->Id);
							int waitingOrderGroupIndex = 0;
							for (list<Vehicle>::iterator localVIter2 = PlanBasedApproach_ramp::LastLockingList.begin(); localVIter2 != PlanBasedApproach_ramp::LastLockingList.end(); localVIter2++) {
								if (localVIter2->WaitingOrder / 10 > waitingOrderGroupIndex) {
									waitingOrderGroupIndex = localVIter2->WaitingOrder / 10;
								}
							}

							DrivingPlan plan = *new DrivingPlan();
							plan = aExistVehicleDrivingPlan[localVIterator->Id];
							plan.MappingVehicleId = localVIterator->ConflictId;
							plan.LeadingVehicleId = localVIterator->HeadId;
							plan.StrategyType = localVIterator->StrategyMode;
							plan.LeadDistList = localVIterator->LeadDistList;
							plan.WaitingOrder = localVIterator->WaitingOrder;
							PlanBasedApproach_ramp::DrivingPlanDict[localVIterator->Id] = plan;
						}
						else {
							PlanBasedApproach_ramp::SimuVehicleDict[localVIterator->Id] = *localVIterator;
							PlanBasedApproach_ramp::SimuIdList.push_back(localVIterator->Id);

							DrivingPlan plan = *new DrivingPlan();
							plan = aExistVehicleDrivingPlan[localVIterator->Id];
							plan.MappingVehicleId = localVIterator->ConflictId;
							plan.LeadingVehicleId = localVIterator->HeadId;
							plan.StrategyType = localVIterator->StrategyMode;
							plan.LeadDistList = localVIterator->LeadDistList;
							plan.WaitingOrder = localVIterator->WaitingOrder;
							PlanBasedApproach_ramp::DrivingPlanDict[localVIterator->Id] = plan;
						}
					}
					else {

						PlanBasedApproach_ramp::SimuVehicleDict[localVIterator->Id] = *localVIterator;
						PlanBasedApproach_ramp::SimuIdList.push_back(localVIterator->Id);

						DrivingPlan plan = *new DrivingPlan();
						plan = aExistVehicleDrivingPlan[localVIterator->Id];
						plan.MappingVehicleId = localVIterator->ConflictId;
						plan.LeadingVehicleId = localVIterator->HeadId;
						plan.StrategyType = localVIterator->StrategyMode;
						plan.LeadDistList = localVIterator->LeadDistList;
						plan.WaitingOrder = localVIterator->WaitingOrder;
						PlanBasedApproach_ramp::DrivingPlanDict[localVIterator->Id] = plan;
					}
				}
			}
		}
	}


	list<Vehicle> localLastLockingListRemove;
	for (list<Vehicle>::iterator localVIter3 = PlanBasedApproach_ramp::LastLockingList.begin(); localVIter3 != PlanBasedApproach_ramp::LastLockingList.end(); localVIter3++) {
		if (PlanBasedApproach_ramp::DrivingPlanDict[localVIter3->Id].GetCount() == 0) {
			localLastLockingListRemove.push_back(*localVIter3);
		}
	}
	for (list<Vehicle>::iterator localVIter4 = localLastLockingListRemove.begin(); localVIter4 != localLastLockingListRemove.end(); localVIter4++) {
		PlanBasedApproach_ramp::DrivingPlanDict.erase(localVIter4->Id);
		for (list<Vehicle>::iterator Viter = PlanBasedApproach_ramp::LastLockingList.begin(); Viter != PlanBasedApproach_ramp::LastLockingList.end(); Viter++) {
			if (localVIter4->Id == Viter->Id) {
				PlanBasedApproach_ramp::LastLockingList.erase(Viter);
				break;
			}
		}
	}

	return judgeNewIn;
}

double PlanBasedApproach_ramp::Distance2RampConflictCenter(Vehicle aVehicle)
{
	double localDistance;
	if (aVehicle.BlockId == 4) {
		localDistance = sqrt((aVehicle.Location.X - PlanBasedApproach_ramp::ConflictBoundaryPointOnRamp.X) * (aVehicle.Location.X - PlanBasedApproach_ramp::ConflictBoundaryPointOnRamp.X) + (aVehicle.Location.Y - PlanBasedApproach_ramp::ConflictBoundaryPointOnRamp.Y) * (aVehicle.Location.Y - PlanBasedApproach_ramp::ConflictBoundaryPointOnRamp.Y));
	}
	else {
		localDistance = sqrt((aVehicle.Location.X - PlanBasedApproach_ramp::ConflictBoundaryPointOnMainRoad.X) * (aVehicle.Location.X - PlanBasedApproach_ramp::ConflictBoundaryPointOnMainRoad.X) + (aVehicle.Location.Y - PlanBasedApproach_ramp::ConflictBoundaryPointOnMainRoad.Y) * (aVehicle.Location.Y - PlanBasedApproach_ramp::ConflictBoundaryPointOnMainRoad.Y));
	}
	return localDistance;
}

void PlanBasedApproach_ramp::GetVehicleLaneIndex(int id, int& firstIndex, int& secondIndex)
{
	Vehicle localVehicle = PlanBasedApproach_ramp::SimuVehicleDict[id];
	firstIndex = localVehicle.FirstLaneId;
	secondIndex = localVehicle.SecondLaneId;
}

void PlanBasedApproach_ramp::GenerateNewPlan()
{
	PlanBasedApproach_ramp::GenerateSchedule();

	clock_t startTime, endTime;
	startTime = clock();
	PlanBasedApproach_ramp::GenerateBestPlan();
	endTime = clock();

	PlanBasedApproach_ramp::CurrRecord->UsedTime_Plan = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	PlanBasedApproach_ramp::RecordList.push_back(*PlanBasedApproach_ramp::CurrRecord);
}

void PlanBasedApproach_ramp::GenerateSchedule()
{
	PlanBasedApproach_ramp::AllSeqList = *new list<list<list<int>>>;
	PlanBasedApproach_ramp::_scheduleTree_ramp = new ScheduleTree_ramp(
		PlanBasedApproach_ramp::DrivingInVehiclesListArray,
		PlanBasedApproach_ramp::ConflictBoundaryPointOnMainRoad,
		PlanBasedApproach_ramp::ConflictBoundaryPointOnRamp,
		PlanBasedApproach_ramp::SimuVehicleDict,
		PlanBasedApproach_ramp::SimulatorTimeCount * PlanBasedApproach_ramp::SimulatorTimeDelta);
	PlanBasedApproach_ramp::_scheduleTree_ramp->Load();
	PlanBasedApproach_ramp::_scheduleTree_ramp->Run();
	PlanBasedApproach_ramp::_scheduleTree_ramp->AddToAllSeqList(PlanBasedApproach_ramp::AllSeqList, PlanBasedApproach_ramp::AppliedPassingOrder);

	for (map<int, Vehicle>::iterator mapIterIntV = PlanBasedApproach_ramp::SimuVehicleDict.begin(); mapIterIntV != PlanBasedApproach_ramp::SimuVehicleDict.end(); mapIterIntV++) {
		mapIterIntV->second.rulePassingOrder = PlanBasedApproach_ramp::_scheduleTree_ramp->SimuVehicleDict[mapIterIntV->first].rulePassingOrder;
	}
}

void PlanBasedApproach_ramp::GenerateBestPlan()
{

	PlanBasedApproach_ramp::Planner = new TrajectoryPlanner(
		PlanBasedApproach_ramp::SimulatorTimeCount,
		0.5,
		PlanBasedApproach_ramp::ConsiderRadius,
		PlanBasedApproach_ramp::DrivingInVehiclesListArray,
		PlanBasedApproach_ramp::LastLockingList,
		PlanBasedApproach_ramp::SimuVehicleDict,
		PlanBasedApproach_ramp::DrivingPlanDict,
		PlanBasedApproach_ramp::ConflictSubzonesLastPassedVehicleArrivalTime,
		PlanBasedApproach_ramp::AppliedPassingOrder
	);

	double minimumDelay = DBL_MAX;
	double simuDelay = 0;
	double estDelay = 0;
	double totalDelay = 0;

	PlanBasedApproach_ramp::CurrRecord->AllPlanCount = 0;

	while (1)	{
		PlanBasedApproach_ramp::CurrRecord->AllPlanCount++;
		list<list<int>> seqList;
		map<int, DrivingPlan> planDictionary;
		for (map<int, DrivingPlan>::iterator mapIter = PlanBasedApproach_ramp::DrivingPlanDict.begin(); mapIter != PlanBasedApproach_ramp::DrivingPlanDict.end(); mapIter++) {
			planDictionary[mapIter->first] = mapIter->second;
		}
		PlanBasedApproach_ramp::Planner->Load(planDictionary, seqList, PlanBasedApproach_ramp::SimuVehicleDict);
		PlanBasedApproach_ramp::Planner->Run(simuDelay, estDelay, false, totalDelay);

		if (totalDelay >= minimumDelay) {
			continue;
		}
		minimumDelay = totalDelay;
		PlanBasedApproach_ramp::Planner->UploadPlanningResult(PlanBasedApproach_ramp::DrivingPlanDict, PlanBasedApproach_ramp::SimuVehicleDict);
		PlanBasedApproach_ramp::DrvingPlan_SimuVehOrderList = PlanBasedApproach_ramp::Planner->PlanningVehicleList;
		PlanBasedApproach_ramp::BestSeqList = seqList;

		break;

	}
}

void PlanBasedApproach_ramp::UpdateRampArrivalTime(list<Vehicle>& Simulator_Vehicles)
{
	for (int i = 0; i < 2; i++) {
		list<Vehicle>::iterator localVIterator = PlanBasedApproach_ramp::DrivingInVehiclesListArray[i].begin();
		for (int j = 0; localVIterator != PlanBasedApproach_ramp::DrivingInVehiclesListArray[i].end(); j++, localVIterator++) {
			if (localVIterator->InIntersectionRadius = VehicleInIntersectionRadius::Outter) {
				continue;
			}

			list<Vehicle>::iterator VIter = Simulator_Vehicles.begin();
			for (; VIter != Simulator_Vehicles.end(); VIter++) {
				if (VIter->Id = localVIterator->Id) {
					break;
				}
			}

			VIter->InIntersectionRadius = localVIterator->InIntersectionRadius;
			VIter->Arrival = localVIterator->Arrival;
			VIter->PlanState = localVIterator->PlanState;

			Vehicle& localVehicle = PlanBasedApproach_ramp::SimuVehicleDict[VIter->Id];
			VIter->AssignedTimeToArriveConfliceZone = localVehicle.AssignedTimeToArriveConfliceZone;
			VIter->MinimumArrivalTime = localVehicle.MinimumArrivalTime;
		}
	}
}

void PlanBasedApproach_ramp::UpdatePlanningResults(list<Vehicle>& Simulator_Vehicles, map<int, DrivingPlan>& Simulator_DrivingPlan)
{
	for (int i = 0; i < 2; i++)	{
		list<Vehicle>::iterator localVIterator = PlanBasedApproach_ramp::DrivingInVehiclesListArray[i].begin();
		for (int j = 0; localVIterator != PlanBasedApproach_ramp::DrivingInVehiclesListArray[i].end(); j++, localVIterator++)		{
			if (localVIterator->InIntersectionRadius == VehicleInIntersectionRadius::Outter) {
				continue;
			}

			list<Vehicle>::iterator VIter = Simulator_Vehicles.begin();
			for (; VIter != Simulator_Vehicles.end(); VIter++) {
				if (VIter->Id == localVIterator->Id) {
					break;
				}
			}

			VIter->InIntersectionRadius = localVIterator->InIntersectionRadius;
			VIter->Arrival = localVIterator->Arrival;
			VIter->PlanState = localVIterator->PlanState;
			VIter->rulePassingOrder = PlanBasedApproach_ramp::SimuVehicleDict[VIter->Id].rulePassingOrder;

			Vehicle& localVehicle = PlanBasedApproach_ramp::SimuVehicleDict[VIter->Id];
			VIter->AssignedTimeToArriveConfliceZone = localVehicle.AssignedTimeToArriveConfliceZone;
			VIter->MinimumArrivalTime = localVehicle.MinimumArrivalTime;
		}
	}

	for (map<int, DrivingPlan>::iterator PIter = PlanBasedApproach_ramp::DrivingPlanDict.begin(); PIter != PlanBasedApproach_ramp::DrivingPlanDict.end(); PIter++) {
		list<Vehicle>::iterator VIter = Simulator_Vehicles.begin();
		for (; VIter != Simulator_Vehicles.end(); VIter++) {
			if (VIter->Id == PIter->first) {
				break;
			}
		}
		VIter->ConflictId = PIter->second.MappingVehicleId;
		VIter->HeadId = PIter->second.LeadingVehicleId;
		VIter->StrategyMode = PIter->second.StrategyType;
		VIter->LeadDistList = PIter->second.LeadDistList;
		VIter->PassingTime = PIter->second.GetCount();
		VIter->WaitingOrder = PIter->second.WaitingOrder;

		Simulator_DrivingPlan[PIter->first] = PIter->second;
	}
}