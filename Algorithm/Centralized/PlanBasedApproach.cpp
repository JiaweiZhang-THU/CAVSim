#include "PlanBasedApproach.h"
#include "PlanBasedApproachParameters.h"

#include "stdio.h"    
#include "stdlib.h"   
#include "time.h" 

PlanBasedApproach::PlanBasedApproach()
{
    PlanBasedApproach::ConsiderAllCut = paraConsiderAllCut;
   for (int i = 0; i < 20; i++)
    {
        for (int j = 0; j < 20; j++)
        {
            PlanBasedApproach::ConflictSubzonesLastPassedVehicleID[i][j] = 0;
            PlanBasedApproach::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = 0;
        }
    }
}

PlanBasedApproach::PlanBasedApproach(int aIntersectionOneWayLanesNum, BPointCoordinate aIntersectionCenter, double aSimulatorTimeCount)
{
    PlanBasedApproach::OuterRadius = paraOutRadius;
    
    PlanBasedApproach::SImulatorTimeDelta = paraDeltaTime1;

    PlanBasedApproach::ConsiderAllCut = paraConsiderAllCut;
    PlanBasedApproach::IntersectionOneWayLanesNum = aIntersectionOneWayLanesNum;
    PlanBasedApproach::IntersectionCenter = aIntersectionCenter;

    PlanBasedApproach::SimulatorTimeCount = aSimulatorTimeCount;

    for (int i = 0; i < 20; i++)
    {
        for (int j = 0; j < 20; j++)
        {
            PlanBasedApproach::ConflictSubzonesLastPassedVehicleID[i][j] = 0;
            PlanBasedApproach::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = 0;
        }
    }

}


PlanBasedApproach::~PlanBasedApproach()
{
    PlanBasedApproach::BestSeqList.clear();
    PlanBasedApproach::RecordList.clear();

    delete PlanBasedApproach::CurrRecord;
    PlanBasedApproach::AllSeqList.clear();

    PlanBasedApproach::SimuVehicleDict.clear();
    PlanBasedApproach::SimuIdList.clear();

    PlanBasedApproach::LastLockingList.clear();
    PlanBasedApproach::ToDeleteList.clear();

    PlanBasedApproach::DrivingPlanDict.clear();

    PlanBasedApproach::PlanRecordsList.clear();
    
    delete PlanBasedApproach::_scheduleTree;
    delete PlanBasedApproach::Planner;
}

void PlanBasedApproach::Load(int aIntersectionOneWayLanesNum, BPointCoordinate aIntersectionCenter, double aSimulatorTimeCount)
{
    PlanBasedApproach::IntersectionOneWayLanesNum = aIntersectionOneWayLanesNum;
    PlanBasedApproach::IntersectionCenter = aIntersectionCenter;
    PlanBasedApproach::SimulatorTimeCount = aSimulatorTimeCount;
}

void PlanBasedApproach::Load(Intersection aIntersection)
{
    for (int i = 0; i < 2 * aIntersection.NumOfOneWayLanes; i++)
    {
        for (int j = 0; j < 2 * aIntersection.NumOfOneWayLanes; j++)
        {
            PlanBasedApproach::ConflictSubzonesLastPassedVehicleID[i][j] = aIntersection.ConflictSubzonesLastPassedVehicleID[i][j];
            PlanBasedApproach::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = aIntersection.ConflictSubzonesLastPassedVehicleArrivalTime[i][j];
        }
    }
}


void PlanBasedApproach::Init()
{
    PlanBasedApproach::ConsiderRadius = paraConsiderRadius;
    PlanBasedApproach::LockRaduius = paraLocakRadius;

	PlanBasedApproach::BestSeqList = *new list<list<int>>();
	PlanBasedApproach::RecordList = *new list<TimingRecord>();

	PlanBasedApproach::CurrRecord = new TimingRecord();
	PlanBasedApproach::AllSeqList = *new list<list<list<int>>>();

	PlanBasedApproach::SimuVehicleDict = *new map<int, Vehicle>();
	PlanBasedApproach::SimuIdList = *new list<int>();

	PlanBasedApproach::_scheduleTree = new ScheduleTree();
	PlanBasedApproach::_scheduleTree->Init();

	PlanBasedApproach::LastLockingList = *new list<Vehicle>();
	PlanBasedApproach::ToDeleteList = *new list<Vehicle>();

	PlanBasedApproach::DrivingPlanDict = *new map<int, DrivingPlan>();
	PlanBasedApproach::Planner = new TrajectoryPlanner();

	PlanBasedApproach::PlanRecordsList = *new list<PlanRecord>();
	PlanBasedApproach::Planner->Init();
}

bool PlanBasedApproach::Run(map<int, DrivingPlan>& aExistVehicleDrivingPlan, bool& HasNewLockedVehicle)
{

    bool localInfo = PlanBasedApproach::UpdateVehicleRegistrationInfo(aExistVehicleDrivingPlan, HasNewLockedVehicle);

    if (localInfo)
    {
        PlanBasedApproach::CurrRecord->UsedTime_Enum = 0;
        PlanBasedApproach::CurrRecord->UsedTime_Split = 0;
        PlanBasedApproach::CurrRecord->UsedTime_Plan = 0;
        PlanBasedApproach::CurrRecord->ConsiderVehicleCount = 0;
        PlanBasedApproach::CurrRecord->PermutationNum = 0;
        PlanBasedApproach::CurrRecord->AllPlanCount = 0;

        PlanBasedApproach::GenerateNewPlan();

        PlanBasedApproach::AlgorInformation = "Spanning tree size: " + PlanBasedApproach::_scheduleTree->GetAllPerListCount();

        PlanRecord record = *new PlanRecord();
        list<Vehicle> localVehicleList;
        for (map<int, Vehicle>::iterator mapViter = PlanBasedApproach::SimuVehicleDict.begin(); mapViter != PlanBasedApproach::SimuVehicleDict.end(); mapViter++)
        {
            localVehicleList.push_back(mapViter->second);
        }
        record.Generate(localVehicleList, PlanBasedApproach::BestSeqList);
        PlanBasedApproach::PlanRecordsList.push_back(record);
    }

    return localInfo;
}



bool PlanBasedApproach::UpdateVehicleRegistrationInfo(map<int, DrivingPlan>& aExistVehicleDrivingPlan, bool& aHasNewLockedVehicle)
{
    bool judgeNewIn = false;

    for (int i = 0; i < 4 * IntersectionOneWayLanesNum; i++)
    {
        list<Vehicle>::iterator localVIterator = PlanBasedApproach::DrivingInVehiclesListArray[i].begin();
        for (int j = 0; localVIterator != PlanBasedApproach::DrivingInVehiclesListArray[i].end(); j++, localVIterator++)
        {
            double localV2IDistance = PlanBasedApproach::Distance2IntersectionCenter(*localVIterator);
            if (localV2IDistance > PlanBasedApproach::ConsiderRadius)
            {
                localVIterator->InIntersectionRadius = VehicleInIntersectionRadius::Outter;
            }
            else
            {
                if (localV2IDistance < PlanBasedApproach::LockRaduius)
                {
                    localVIterator->InIntersectionRadius = VehicleInIntersectionRadius::Inner;
                }
                else
                {
                    localVIterator->InIntersectionRadius = VehicleInIntersectionRadius::Middle;
                }
            }

            if (localV2IDistance < PlanBasedApproach::ConsiderRadius)
            {
                if (localVIterator->PlanState == VehiclePlanState::Free)
                {
                    localVIterator->Arrival = PlanBasedApproach::SimulatorTimeCount;
                    localVIterator->PlanState = VehiclePlanState::Allocated;
                    judgeNewIn = true;
                    PlanBasedApproach::SimuVehicleDict[localVIterator->Id] = *localVIterator;
                    PlanBasedApproach::SimuIdList.push_back(localVIterator->Id);

                    DrivingPlan plan = *new DrivingPlan();
                    PlanBasedApproach::DrivingPlanDict[localVIterator->Id] = plan;
                }
                else
                {
                    if (localVIterator->PlanState == VehiclePlanState::Allocated)
                    {
                        if (localV2IDistance < PlanBasedApproach::LockRaduius)
                        {
                            Vehicle inLockSimuVehicle = SimuVehicleDict[localVIterator->Id];
                            localVIterator->PlanState = VehiclePlanState::Locaked;
                            aHasNewLockedVehicle = true;
                            PlanBasedApproach::SimuVehicleDict.erase(localVIterator->Id);
                            PlanBasedApproach::LastLockingList.push_back(inLockSimuVehicle);
                            int waitingOrderGroupIndex = 0;
                            for (list<Vehicle>::iterator localVIter2 = PlanBasedApproach::LastLockingList.begin(); localVIter2 != PlanBasedApproach::LastLockingList.end(); localVIter2++)
                            {
                                if (localVIter2->WaitingOrder / 10 > waitingOrderGroupIndex)
                                {
                                    waitingOrderGroupIndex = localVIter2->WaitingOrder / 10;
                                }
                            }

                            for (int i = 0; i < localVIterator->TrajectoryCoverageConflictSubzonesArraySize; i++)
                            {
                                int localConflictSubzone = localVIterator->TrajectoryCoverageConflictSubzonesArray[i] - 1;

                                int Xindex = (int)localConflictSubzone / (2 * paraLaneNum5);
                                int Yindex = (int)localConflictSubzone % (int)(2 * paraLaneNum5);

                                PlanBasedApproach::ConflictSubzonesLastPassedVehicleArrivalTime[Xindex][Yindex] = max(
                                    PlanBasedApproach::ConflictSubzonesLastPassedVehicleArrivalTime[Xindex][Yindex], localVIterator->AssignedArrivalTimeToConflictSubzones[i]
                                );
                                
                            }


                            DrivingPlan plan = *new DrivingPlan();

                            plan = aExistVehicleDrivingPlan[localVIterator->Id];

                            plan.MappingVehicleId = localVIterator->ConflictId;
                            plan.LeadingVehicleId = localVIterator->HeadId;
                            plan.StrategyType = localVIterator->StrategyMode;
                            plan.LeadDistList = localVIterator->LeadDistList;

                            plan.WaitingOrder = localVIterator->WaitingOrder;
                            PlanBasedApproach::DrivingPlanDict[localVIterator->Id] = plan;
                        }
                        else
                        {
                            PlanBasedApproach::SimuVehicleDict[localVIterator->Id] = *localVIterator;
                            PlanBasedApproach::SimuIdList.push_back(localVIterator->Id);

                            DrivingPlan plan = *new DrivingPlan();

                            plan = aExistVehicleDrivingPlan[localVIterator->Id];

                            plan.MappingVehicleId = localVIterator->ConflictId;
                            plan.LeadingVehicleId = localVIterator->HeadId;
                            plan.StrategyType = localVIterator->StrategyMode;
                            plan.LeadDistList = localVIterator->LeadDistList;

                            plan.WaitingOrder = localVIterator->WaitingOrder;
                            PlanBasedApproach::DrivingPlanDict[localVIterator->Id] = plan;
                        }
                    }
                    else
                    {
                        PlanBasedApproach::SimuVehicleDict[localVIterator->Id] = *localVIterator;
                        PlanBasedApproach::SimuIdList.push_back(localVIterator->Id);

                        DrivingPlan plan = *new DrivingPlan();

                        plan = aExistVehicleDrivingPlan[localVIterator->Id];

                        plan.MappingVehicleId = localVIterator->ConflictId;
                        plan.LeadingVehicleId = localVIterator->HeadId;
                        plan.StrategyType = localVIterator->StrategyMode;
                        plan.LeadDistList = localVIterator->LeadDistList;

                        plan.WaitingOrder = localVIterator->WaitingOrder;
                        PlanBasedApproach::DrivingPlanDict[localVIterator->Id] = plan;
                    }
                }
            }
        }
    }

    list<Vehicle> localLastLockingListRemove;
    for (list<Vehicle>::iterator localVIter3 = PlanBasedApproach::LastLockingList.begin(); localVIter3 != PlanBasedApproach::LastLockingList.end(); localVIter3++)
    {
        if (PlanBasedApproach::DrivingPlanDict[localVIter3->Id].GetCount() == 0)
        {
            localLastLockingListRemove.push_back(*localVIter3);
        }
    }
    for (list<Vehicle>::iterator localVIter4 = localLastLockingListRemove.begin(); localVIter4 != localLastLockingListRemove.end(); localVIter4++)
    {
        PlanBasedApproach::DrivingPlanDict.erase(localVIter4->Id);
        for (list<Vehicle>::iterator Viter = PlanBasedApproach::LastLockingList.begin(); Viter != PlanBasedApproach::LastLockingList.end(); Viter++)
        {
            if (localVIter4->Id == Viter->Id)
            {
                PlanBasedApproach::LastLockingList.erase(Viter);
                break;
            }
        }
    }

    return judgeNewIn;
}


double PlanBasedApproach::Distance2IntersectionCenter(Vehicle aVehicle)
{
    double localDistance = sqrt((aVehicle.Location.X - PlanBasedApproach::IntersectionCenter.X) * (aVehicle.Location.X - PlanBasedApproach::IntersectionCenter.X) + (aVehicle.Location.Y - PlanBasedApproach::IntersectionCenter.Y) * (aVehicle.Location.Y - PlanBasedApproach::IntersectionCenter.Y));
    return localDistance;
}


void PlanBasedApproach::GetVehicleLaneIndex(int id, int& firstIndex, int& secondIndex)
{
    Vehicle localVehicle = PlanBasedApproach::SimuVehicleDict[id];
    firstIndex = localVehicle.FirstLaneId;
    secondIndex = localVehicle.SecondLaneId;
}


void PlanBasedApproach::GenerateNewPlan()
{

    PlanBasedApproach::GenerateSchedule();


    clock_t startTime, endTime;
    startTime = clock();

    PlanBasedApproach::GenerateBestPlan();

    endTime = clock();

    PlanBasedApproach::CurrRecord->UsedTime_Plan = (double)(endTime - startTime) / CLOCKS_PER_SEC;

    PlanBasedApproach::RecordList.push_back(*PlanBasedApproach::CurrRecord);
    
}


void PlanBasedApproach::GenerateSchedule()
{
    PlanBasedApproach::AllSeqList = *new list<list<list<int>>>;
    PlanBasedApproach::_scheduleTree = new ScheduleTree(
        PlanBasedApproach::DrivingInVehiclesListArray, 
        PlanBasedApproach::IntersectionOneWayLanesNum, 
        PlanBasedApproach::IntersectionCenter,
        PlanBasedApproach::SimuVehicleDict,
        PlanBasedApproach::ConflictSubzonesLastPassedVehicleArrivalTime,
        PlanBasedApproach::SimulatorTimeCount*PlanBasedApproach::SImulatorTimeDelta);
    PlanBasedApproach::_scheduleTree->Load();
    PlanBasedApproach::_scheduleTree->Run();

    PlanBasedApproach::_scheduleTree->AddToAllSeqList(PlanBasedApproach::AllSeqList,PlanBasedApproach::AppliedPassingOrder);

    for (map<int, Vehicle>::iterator mapIterIntV = PlanBasedApproach::SimuVehicleDict.begin(); mapIterIntV != PlanBasedApproach::SimuVehicleDict.end(); mapIterIntV++)
    {
        mapIterIntV->second.DRPassingOrder = PlanBasedApproach::_scheduleTree->SimuVehicleDict[mapIterIntV->first].DRPassingOrder;
    }
    
}


void PlanBasedApproach::GenerateBestPlan()
{
    PlanBasedApproach::Planner = new TrajectoryPlanner(
        PlanBasedApproach::SimulatorTimeCount,
        PlanBasedApproach::IntersectionOneWayLanesNum,
        PlanBasedApproach::OuterRadius,
        PlanBasedApproach::DrivingInVehiclesListArray,
        PlanBasedApproach::LastLockingList,
        PlanBasedApproach::SimuVehicleDict,
        PlanBasedApproach::DrivingPlanDict,
        PlanBasedApproach::ConflictSubzonesLastPassedVehicleArrivalTime,
        PlanBasedApproach::AppliedPassingOrder
    );

    double minimumDelay = DBL_MAX;

    double simuDelay = 0;
    double estDelay = 0;

    double totalDelay = 0;

    PlanBasedApproach::CurrRecord->AllPlanCount = 0;

    
    while(1)
    {
        PlanBasedApproach::CurrRecord->AllPlanCount++;
        list<list<int>> seqList;

        map<int, DrivingPlan> planDictionary;
        for (map<int, DrivingPlan>::iterator mapIter = PlanBasedApproach::DrivingPlanDict.begin(); mapIter != PlanBasedApproach::DrivingPlanDict.end(); mapIter++)
        {
            planDictionary[mapIter->first] = mapIter->second;
        }

        PlanBasedApproach::Planner->Load(planDictionary, seqList, PlanBasedApproach::SimuVehicleDict);
        PlanBasedApproach::Planner->Run(simuDelay, estDelay, false, totalDelay);

        if (totalDelay >= minimumDelay)
        {
            continue;
        }
        minimumDelay = totalDelay;
        PlanBasedApproach::Planner->UploadPlanningResult(PlanBasedApproach::DrivingPlanDict, PlanBasedApproach::SimuVehicleDict);
        PlanBasedApproach::DrivingPlan_SimuVehOrderList = PlanBasedApproach::Planner->PlanningVehicleList;

        PlanBasedApproach::BestSeqList = seqList;
        break;
    }

    
}


void PlanBasedApproach::UpdateIntersectionArrivalTime(list<Vehicle>& Simulator_Vehicles, Intersection& aIntersection)
{
    for (int i = 0; i < 4 * IntersectionOneWayLanesNum; i++)
    {
        list<Vehicle>::iterator localVIterator = PlanBasedApproach::DrivingInVehiclesListArray[i].begin();
        for (int j = 0; localVIterator != PlanBasedApproach::DrivingInVehiclesListArray[i].end(); j++, localVIterator++)
        {
            if (localVIterator->InIntersectionRadius == VehicleInIntersectionRadius::Outter)
            {
                continue;
            }

            list<Vehicle>::iterator VIter = Simulator_Vehicles.begin();
            for (; VIter != Simulator_Vehicles.end(); VIter++)
            {
                if (VIter->Id == localVIterator->Id)
                {
                    break;
                }
            }

            VIter->InIntersectionRadius = localVIterator->InIntersectionRadius;
            VIter->Arrival = localVIterator->Arrival;
            VIter->PlanState = localVIterator->PlanState;

            Vehicle& localVehicle = PlanBasedApproach::SimuVehicleDict[VIter->Id];
            for (int i = 0; i < localVehicle.TrajectoryCoverageConflictSubzonesArraySize; i++)
            {
                VIter->AssignedArrivalTimeToConflictSubzones[i] = localVehicle.AssignedArrivalTimeToConflictSubzones[i];
                VIter->MinimumArrivalTimeToConflictSubzones[i] = localVehicle.MinimumArrivalTimeToConflictSubzones[i];
            }
            VIter->AssignedTimeToArriveConfliceZone = localVehicle.AssignedTimeToArriveConfliceZone;
            VIter->MinimumArrivalTime = localVehicle.MinimumArrivalTime;

        }
    }

    for (int i = 0; i < 2 * aIntersection.NumOfOneWayLanes; i++)
    {
        for (int j = 0; j < 2 * aIntersection.NumOfOneWayLanes; j++)
        {
            aIntersection.ConflictSubzonesLastPassedVehicleID[i][j] = PlanBasedApproach::ConflictSubzonesLastPassedVehicleID[i][j];
            aIntersection.ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = PlanBasedApproach::ConflictSubzonesLastPassedVehicleArrivalTime[i][j];
        }
    }
}

void PlanBasedApproach::UpdatePlanningResults(list<Vehicle>& Simulator_Vehicles, map<int, DrivingPlan>& Simulator_DrivingPLan)
{
    for (int i = 0; i < 4 * IntersectionOneWayLanesNum; i++)
    {
        list<Vehicle>::iterator localVIterator = PlanBasedApproach::DrivingInVehiclesListArray[i].begin();
        for (int j = 0; localVIterator != PlanBasedApproach::DrivingInVehiclesListArray[i].end(); j++, localVIterator++)
        {
            if (localVIterator->InIntersectionRadius == VehicleInIntersectionRadius::Outter)
            {
                continue;
            }

            list<Vehicle>::iterator VIter = Simulator_Vehicles.begin();
            for (; VIter != Simulator_Vehicles.end(); VIter++)
            {
                if (VIter->Id == localVIterator->Id)
                {
                    break;
                }
            }

            VIter->InIntersectionRadius = localVIterator->InIntersectionRadius;
            VIter->Arrival = localVIterator->Arrival;
            VIter->PlanState = localVIterator->PlanState;
            VIter->DRPassingOrder = PlanBasedApproach::SimuVehicleDict[VIter->Id].DRPassingOrder;

            Vehicle& localVehicle = PlanBasedApproach::SimuVehicleDict[VIter->Id];
            for (int i = 0; i < localVehicle.TrajectoryCoverageConflictSubzonesArraySize; i++)
            {
                VIter->AssignedArrivalTimeToConflictSubzones[i] = localVehicle.AssignedArrivalTimeToConflictSubzones[i];
                VIter->MinimumArrivalTimeToConflictSubzones[i] = localVehicle.MinimumArrivalTimeToConflictSubzones[i];
            }
            VIter->AssignedTimeToArriveConfliceZone = localVehicle.AssignedTimeToArriveConfliceZone;
            VIter->MinimumArrivalTime = localVehicle.MinimumArrivalTime;

        }
    }

    for (map<int, DrivingPlan>::iterator PIter = PlanBasedApproach::DrivingPlanDict.begin(); PIter != PlanBasedApproach::DrivingPlanDict.end(); PIter++)
    {
        list<Vehicle>::iterator VIter = Simulator_Vehicles.begin();
        for (; VIter != Simulator_Vehicles.end(); VIter++)
        {
            if (VIter->Id == PIter->first)
            {
                break;
            }
        }
        VIter->ConflictId = PIter->second.MappingVehicleId;
        VIter->HeadId = PIter->second.LeadingVehicleId;
        VIter->StrategyMode = PIter->second.StrategyType;
        VIter->LeadDistList = PIter->second.LeadDistList;
        VIter->PassingTime = PIter->second.GetCount();
        VIter->WaitingOrder = PIter->second.WaitingOrder;

        Simulator_DrivingPLan[PIter->first] = PIter->second;
    }


}

void PlanBasedApproach::FollowAllocatedPlan()
{
}

