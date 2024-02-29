#include "simulatorSingleIntersection.h"
#include "SimulatorSingleIntersectionParameters.h"
#include "stdio.h"    
#include "stdlib.h"   
#include "time.h" 

Simulator1::Simulator1()
{
	Simulator1::SimulatingStep = 0;
	Simulator1::SimulatingStep2Time = 0;
	Simulator1::TimeStep = paraTimeStep1;
	Simulator1::TimeIntervalForUpdateTrafficFlowRate = paraTimeIntervalForUpdateTrafficFlowRate1;

	Simulator1::VehicleNum = 0;

	Simulator1::InitialStatisticalVariable();

	Simulator1::VehileFlowPossionLambda = paraVehileFlowPossionLambda1;

	Simulator1::SimulatorId = rand() % 1000;

	Simulator1::RadiusOfIntersectionControlZone = paraRadiusOfIntersectionControlZone;
}

list<int> Simulator1::VehicleFlowGenerator(int aVehicleNum, int aBeginTime)
{
	list<double> TimeInterval;
	list<double>::iterator localDoubleIter;
	for (int i = 0; i < aVehicleNum; i++)
	{
		TimeInterval.push_back(Simulator1::RandomExponential());
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

list<int> Simulator1::VehicleFlowReadFromFile()
{
	string localFile = "ArrRateFlow_" + to_string(int(Simulator1::VehileFlowPossionLambda * (3600 * 10 / 8))) + ".txt";
	ifstream inFile;
	inFile.open(localFile);
	list<int> VehicleFlow;
	int x;
	while (inFile >> x) {
		VehicleFlow.push_back(x);
	}
	return VehicleFlow;
}

BNode Simulator1::InitialBNodeForNewVehicle()
{
	list<BNode> localEntranceOnRoadNetwork = Simulator1::SingleIntersectionInstance.GetEnteringVehicleInitialPositonAndBoseList();
	int EntranceNum = int(localEntranceOnRoadNetwork.size());
	int RandomEntrance = Simulator1::VehicleIdIterator % EntranceNum;
	list<BNode>::iterator localIter = localEntranceOnRoadNetwork.begin();
	for (int i = 0; i < RandomEntrance; i++)
	{
		localIter++;
	}
	return *localIter;
}


bool Simulator1::CreateNewVehicle(int aNewVehicleId, Vehicle& NewVehicle)
{
	BNode InitialBNodeForNewVehicle = Simulator1::InitialBNodeForNewVehicle();
	NewVehicle.Location = InitialBNodeForNewVehicle.Location;
	NewVehicle.PoseAngle = 0;
	NewVehicle.Direction = InitialBNodeForNewVehicle.Direction;
	NewVehicle.Id = aNewVehicleId;
	NewVehicle.RoadBlockType = InitialBNodeForNewVehicle.RoadBlockType;
	NewVehicle.Type = (int)Simulator1::MyCls_Random.randomUniform(0, 4);
	NewVehicle.TimeStep = Simulator1::TimeStep;
	NewVehicle.BornTime = Simulator1::SimulatingStep;
	NewVehicle.Speed = NewVehicle.MaxStraightSpeed;
	if (Simulator1::SingleIntersectionInstance.Intersection0.IsInIntersection(InitialBNodeForNewVehicle.Location) == true)
	{
		NewVehicle.RoadId = Simulator1::SingleIntersectionInstance.Intersection0.Id;
		NewVehicle.BlockId = InitialBNodeForNewVehicle.IntersectionZoneType;
		NewVehicle.LaneId = InitialBNodeForNewVehicle.Id;
		NewVehicle.LaneUnitPointer = Simulator1::SingleIntersectionInstance.Intersection0.FindLaneUnitPointer(NewVehicle.BlockId, NewVehicle.LaneId);
		NewVehicle.IntersectionPointer = &Simulator1::SingleIntersectionInstance.Intersection0;
		NewVehicle.LaneUnitPointer->AddVehicle(NewVehicle);
		NewVehicle.IntersectionPointer->AddVehicle(NewVehicle);
		NewVehicle.EnteringNode = Simulator1::SingleIntersectionInstance.OriginalRoadNodePassingIntersection(NewVehicle);
		NewVehicle.TargetNode = Simulator1::SingleIntersectionInstance.NextRoadNodePassingIntersection(NewVehicle); 
		NewVehicle.IntersectionPointer->VehicleTrajectoryCoverageConflictSubzones(NewVehicle);
		NewVehicle.LeftLaneDistance = Simulator1::SingleIntersectionInstance.Intersection0.IntersectionStraightBlockLength;
		NewVehicle.IntersectionPointer->StraightLaneVehiclesAN[NewVehicle.BlockId - 1]++;
		return true;
	}
	return true;
}

double Simulator1::RandomExponential()
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
	ExponentialValue = (-1.0 / Simulator1::VehileFlowPossionLambda) * log(1 - ExponentialValue);
	return ExponentialValue;
}

void Simulator1::ClearVehicle()
{
	Simulator1::VehicleList.clear();
}

void Simulator1::UpdateVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = Simulator1::VehicleList.begin(); localVehicleIterator != Simulator1::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			*localVehicleIterator = aVehicle;
		}
	}
}

void Simulator1::AddVehicle(Vehicle aVehicle)
{
	Simulator1::VehicleList.push_back(aVehicle);
}

void Simulator1::RemoveVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = Simulator1::VehicleList.begin(); localVehicleIterator != Simulator1::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			Simulator1::VehicleList.erase(localVehicleIterator);
			break;
		}
	}
}

void Simulator1::SetVehiclePointerOnStraightLaneUnit(Vehicle& aVehicle, StraightLane& aStraightLane)
{
	int BlockId = aVehicle.BlockId;
	int LaneUnitId = aVehicle.LaneId;
	list<StraightLaneBlock>::iterator localBlockIterator;
	list<LaneUnit>::iterator localLaneUnitIterator;

	int i = 0;
	for (localBlockIterator = aStraightLane.StraightLaneList.begin(); localBlockIterator != aStraightLane.StraightLaneList.end(); localBlockIterator++)
	{
		i += 1;
		if (i == BlockId)
		{
			break;
		}
	}

	int j = 0;
	for (localLaneUnitIterator = localBlockIterator->StraightMultiLanes.begin(); localLaneUnitIterator != localBlockIterator->StraightMultiLanes.end(); localLaneUnitIterator++)
	{
		j += 1;
		if (j == LaneUnitId)
		{
			break;
		}
	}

	aVehicle.LaneUnitPointer = &(*localLaneUnitIterator);
}

bool Simulator1::VehicleAreaPositioningUpdate(Vehicle& aVehicle)
{
	if (Simulator1::SingleIntersectionInstance.Intersection0.IsInIntersection(aVehicle.Location) == true)
	{
		aVehicle.RoadId = Simulator1::SingleIntersectionInstance.Intersection0.Id;
		aVehicle.RoadBlockType = 2;
		aVehicle.IntersectionPointer = &Simulator1::SingleIntersectionInstance.Intersection0;
		aVehicle.IntersectionPointer->AddVehicle(aVehicle);
		aVehicle.BlockId = aVehicle.IntersectionPointer->WhichAreaInIntersection(aVehicle.Location);
		aVehicle.LaneUnitPointer = Simulator1::SingleIntersectionInstance.Intersection0.FindLaneUnitPointer(aVehicle.BlockId, aVehicle.LaneId);
		aVehicle.LaneUnitPointer->AddVehicle(aVehicle);
		aVehicle.EnteringNode = Simulator1::SingleIntersectionInstance.OriginalRoadNodePassingIntersection(aVehicle);
		aVehicle.TargetNode = Simulator1::SingleIntersectionInstance.NextRoadNodePassingIntersection(aVehicle);
		aVehicle.IntersectionPointer->VehicleTrajectoryCoverageConflictSubzones(aVehicle);
		aVehicle.IntersectionPointer->StraightLaneVehiclesAN[aVehicle.BlockId - 1]++;

		return true;
	}
	return false;
}

bool Simulator1::IsInOriginalBlock(Vehicle& aVehicle)
{
	if (aVehicle.RoadBlockType == 0)
	{
		return false;
	}
	else
	{
		if (aVehicle.RoadBlockType == 1)
		{
			bool localPosotionBool = aVehicle.StraightLanePointer->StraightRectangle.IsPointInRect(aVehicle.Location.X, aVehicle.Location.Y);
			if (localPosotionBool == false)
			{
				aVehicle.LaneUnitPointer->RemoveVehicle(aVehicle);
			}
			return localPosotionBool;
		}
		else
		{
			if (aVehicle.RoadBlockType == 2)
			{
				bool localPositionBool = aVehicle.IntersectionPointer->IsInIntersection(aVehicle.Location);
				if (localPositionBool == false)
				{
					aVehicle.LaneUnitPointer->RemoveVehicle(aVehicle);
					aVehicle.IntersectionPointer->RemoveVehicle(aVehicle);
					aVehicle.IntersectionPointer->RemoveVehicleInCrossingArea(aVehicle);
				}
				return localPositionBool;
			}
			else
			{
				return false;
			}
		}
	}
}

void Simulator1::UpdateVehicle2StraightLane(Vehicle& aVehicle)
{
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
				aVehicle.BlockId = aVehicle.LaneUnitPointer->BlockId;
				aVehicle.LaneId = aVehicle.LaneUnitPointer->Id;
				break;
			}
		}
		if (localFlagIsAdjacentUnitFound == false)
		{
			aVehicle.LaneUnitPointer->RemoveVehicle(aVehicle);
		}
	}
}

void Simulator1::IsVehickEnteringIntersectionControlZone(Vehicle& aVehicle)
{
	if (aVehicle.TimeOfEnteringIntersectionCircleControlZone == -1)
	{
		double distance2IntersectionCenter = sqrt((aVehicle.Location.X - aVehicle.IntersectionPointer->Location.X) * (aVehicle.Location.X - aVehicle.IntersectionPointer->Location.X)
			+ (aVehicle.Location.Y - aVehicle.IntersectionPointer->Location.Y) * (aVehicle.Location.Y - aVehicle.IntersectionPointer->Location.Y));
		if (distance2IntersectionCenter < Simulator1::RadiusOfIntersectionControlZone)
		{
			aVehicle.TimeOfEnteringIntersectionCircleControlZone = Simulator1::SimulatingStep;
		}
	}

}

void Simulator1::InitialStatisticalVariable()
{
	Simulator1::SingleIntersectionInstance.Intersection0.InitialStatisticalVariable();
}

void Simulator1::UpdateVehicle2Intersection(Vehicle& aVehicle)
{
	aVehicle.IntersectionPointer->UpdateVehicle(aVehicle);
	Simulator1::IsVehickEnteringIntersectionControlZone(aVehicle);
	if (aVehicle.BlockId == aVehicle.IntersectionPointer->WhichAreaInIntersection(aVehicle.Location))
	{
		if (aVehicle.BlockId == 0)
		{
			aVehicle.IntersectionPointer->UpdateVehicleInCrossingArea(aVehicle);
		}
		else
		{
			aVehicle.LaneUnitPointer->UpdateVehicle(aVehicle);
		}
	}
	else
	{
		if (aVehicle.BlockId != 0 && aVehicle.IntersectionPointer->WhichAreaInIntersection(aVehicle.Location) == 0)
		{
			aVehicle.BlockId = aVehicle.IntersectionPointer->WhichAreaInIntersection(aVehicle.Location);
			aVehicle.LaneUnitPointer->RemoveVehicle(aVehicle);
			aVehicle.IntersectionPointer->AddVehicleInCrossingArea(aVehicle);
			aVehicle.TimeOfEnteringIntersectionConflictZone = Simulator1::SimulatingStep;
			aVehicle.TimeCostToTargetNede = aVehicle.GetDistance(aVehicle.TargetNode) / aVehicle.MaxSteerSpeed;
			aVehicle.IntersectionPointer->PassingIntersectionVehiclesNum++;
		}
		else
		{
			if (aVehicle.BlockId == 0 && aVehicle.IntersectionPointer->WhichAreaInIntersection(aVehicle.Location) != 0)
			{
				aVehicle.BlockId = aVehicle.IntersectionPointer->WhichAreaInIntersection(aVehicle.Location);
				aVehicle.IntersectionPointer->RemoveVehicleInCrossingArea(aVehicle);
				aVehicle.LaneUnitPointer = aVehicle.IntersectionPointer->FindLaneUnitPointer(aVehicle.BlockId, aVehicle.LaneId);
				aVehicle.LaneUnitPointer->AddVehicle(aVehicle);
				aVehicle.Location = aVehicle.TargetNode.Location;
				aVehicle.Direction = aVehicle.LaneUnitPointer->Direction;
				aVehicle.PoseAngle = 0;
				aVehicle.PlanState = VehiclePlanState::Free;
				aVehicle.InIntersectionRadius = VehicleInIntersectionRadius::Outter;
				aVehicle.IntersectionPointer->StraightLaneVehiclesAN[aVehicle.BlockId - 1]++;

			}
		}

	}
}

void Simulator1::UpdateVehicle2MapBlock(Vehicle& aVehicle)
{
	if (Simulator1::IsInOriginalBlock(aVehicle) == true)
	{
		if (aVehicle.RoadBlockType == 1)
		{
			Simulator1::UpdateVehicle2StraightLane(aVehicle);
		}
		else
		{
			if (aVehicle.RoadBlockType == 2)
			{
				Simulator1::UpdateVehicle2Intersection(aVehicle);
			}
		}
	}
	else
	{
		if (Simulator1::SingleIntersectionInstance.IsInGriddedMap(aVehicle.Location) == false)
		{
			Simulator1::VehicleOverList.push_back(aVehicle);
		}
		else
		{
			Simulator1::VehicleAreaPositioningUpdate(aVehicle);

		}
	}
}

bool Simulator1::IsLaneChangingPermitted(Vehicle aVehicel, int aLaneChangeAction)
{
	if (aVehicel.BlockId == aVehicel.StraightLanePointer->StraightLaneBlockNum)
	{
		return false; 
	}

	if (aVehicel.LaneId > 1 && aVehicel.LaneId < paraDefaultLaneNumDefinedSLBP1)
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
			if (aVehicel.LaneId == paraDefaultLaneNumDefinedSLBP1 && aLaneChangeAction == 1)
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

list<Vehicle> Simulator1::PerceptionSurroundingVehicles(Vehicle& aVehicle)
{
	list<Vehicle> SurroundingVehicleList;
	if (aVehicle.RoadBlockType != 1 && aVehicle.RoadBlockType != 2)
	{
		return SurroundingVehicleList;
	}
	LaneUnit* localLaneUnitPointer = NULL;

	if (aVehicle.RoadBlockType == 1)
	{
		for (int i = 0; i < 9; i++)
		{
			localLaneUnitPointer = aVehicle.LaneUnitPointer->AdjacentUnitPointer[i];
			if (localLaneUnitPointer == NULL)
			{
				continue;
			}
			for (list<Vehicle>::iterator localVehicleIterator = localLaneUnitPointer->VehicleList.begin(); localVehicleIterator != localLaneUnitPointer->VehicleList.end(); localVehicleIterator++)
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

	if (aVehicle.RoadBlockType == 2)
	{
		if (aVehicle.BlockId != 0)
		{

			int j = 1;
			list<LaneUnit>::iterator localLUIter = aVehicle.IntersectionPointer->StraightLaneInterface[aVehicle.BlockId - 1].StraightMultiLanes.begin();
			for (; localLUIter != aVehicle.IntersectionPointer->StraightLaneInterface[aVehicle.BlockId - 1].StraightMultiLanes.end(); localLUIter++, j++)
			{
				if (j == aVehicle.LaneId)
				{
					break;
				}
			}
			return localLUIter->VehicleList;
		}
		else
		{
			return aVehicle.IntersectionPointer->VehicleList;
		}
	}
	return SurroundingVehicleList;
}

bool Simulator1::CooperativeDrivingAtIntersection()
{
	Simulator1::MyPlanBasedApproach = new PlanBasedApproach(Simulator1::SingleIntersectionInstance.Intersection0.NumOfOneWayLanes, Simulator1::SingleIntersectionInstance.Intersection0.Location, Simulator1::SimulatingStep);
	Simulator1::MyPlanBasedApproach->Init();
	Simulator1::MyPlanBasedApproach->Load(Simulator1::SingleIntersectionInstance.Intersection0);

	for (int i = 0; i < 4; i++)
	{
		int j = 0;
		
		for (list<LaneUnit>::iterator localLUIter = Simulator1::SingleIntersectionInstance.Intersection0.StraightLaneInterface[2 * i].StraightMultiLanes.begin(); localLUIter != Simulator1::SingleIntersectionInstance.Intersection0.StraightLaneInterface[2 * i].StraightMultiLanes.end(); localLUIter++,j++)
		{
			Simulator1::MyPlanBasedApproach->DrivingInVehiclesListArray[paraDefaultLaneNumDefinedSLBP1 * i + j].clear();
			if (localLUIter->VehicleList.size() == 0)
			{
				continue;
			}
			for (list<Vehicle>::iterator VIter = localLUIter->VehicleList.begin(); VIter != localLUIter->VehicleList.end(); VIter++)
			{
				Simulator1::MyPlanBasedApproach->DrivingInVehiclesListArray[paraDefaultLaneNumDefinedSLBP1 * i + j].push_back(*VIter);
			}
		}
	}

	bool HasNewLockedVehicle = false;
	bool localInfo = Simulator1::MyPlanBasedApproach->Run(Simulator1::VehicleDrivingPlan, HasNewLockedVehicle);
	if (localInfo == true)
	{
		Simulator1::MyPlanBasedApproach->UpdatePlanningResults(Simulator1::VehicleList,Simulator1::VehicleDrivingPlan);
	}
	if (HasNewLockedVehicle == true)
	{
		Simulator1::MyPlanBasedApproach->UpdateIntersectionArrivalTime(Simulator1::VehicleList,Simulator1::SingleIntersectionInstance.Intersection0);
	}

	return localInfo;
}

void Simulator1::Run()
{
	list<int> VehicleArrivingTimetable = Simulator1::VehicleFlowGenerator(paraPerFlowVehicleNum1, 0);
	list<int>::iterator localVehicleArrivingTimePoint = VehicleArrivingTimetable.begin();

	Simulator1::SimulatingStep = 0;
	Simulator1::SimulatingStep2Time = 0;
	Simulator1::VehicleIdIterator = 0;

	ofstream localWritingIn;
	int RandomNameValue = Simulator1::SimulatorId;
	localWritingIn.open("Data/Intersection_" + to_string(Simulator1::SimulatorId) + ".txt", ios::out);

	Vehicle NewVehicle;

	int localSchedualTimeInterval = 20;

	while (true)
	{

		Simulator1::SimulatingStep += 1;
		Simulator1::SimulatingStep2Time += Simulator1::TimeStep;

		if (Simulator1::SimulatingStep % 10 == 0)
		{
			cout << "--------------------- Simulation ID: "<< Simulator1::SimulatorId<<" --- Run Step: " << Simulator1::SimulatingStep << " --- Time: " << Simulator1::SimulatingStep2Time << " s --------------------------------" << endl;
		}

		if (Simulator1::SimulatingStep > *localVehicleArrivingTimePoint)
		{
			Simulator1::VehicleIdIterator += 1;
			if (*localVehicleArrivingTimePoint == VehicleArrivingTimetable.back())
			{
				VehicleArrivingTimetable = Simulator1::VehicleFlowGenerator(paraPerFlowVehicleNum1, Simulator1::SimulatingStep);
				localVehicleArrivingTimePoint = VehicleArrivingTimetable.begin();
			}
			else
			{
				localVehicleArrivingTimePoint++;
			}

			bool CreatSuccess = Simulator1::CreateNewVehicle(Simulator1::VehicleIdIterator, Simulator1::VehicleArray[Simulator1::VehicleNum % 5000]);
			if (CreatSuccess == true)
			{
				Simulator1::VehicleList.push_back(Simulator1::VehicleArray[Simulator1::VehicleNum % 5000]);
				Simulator1::VehicleNum += 1;
			}

		}

		list<Vehicle> localPerceptionSurroundingVehiclesList;
		list<Vehicle>::iterator localVehichIterator;

		if ((Simulator1::SimulatingStep + 1) % localSchedualTimeInterval == 0)
		{
			for (localVehichIterator = Simulator1::VehicleList.begin(); localVehichIterator != Simulator1::VehicleList.end(); localVehichIterator++)
			{
				localPerceptionSurroundingVehiclesList = Simulator1::PerceptionSurroundingVehicles(*localVehichIterator);
				localVehichIterator->UpdatePerceptionSurroundingIntersectionVehicles(localPerceptionSurroundingVehiclesList, *localVehichIterator);
				Simulator1::UpdateVehicle2MapBlock(*localVehichIterator);
			}
			bool IsPlanning = Simulator1::CooperativeDrivingAtIntersection();
		}

		for (Simulator1::VehicleOverList.clear(), localVehichIterator = Simulator1::VehicleList.begin(); localVehichIterator != Simulator1::VehicleList.end(); localVehichIterator++)
		{

			localPerceptionSurroundingVehiclesList = Simulator1::PerceptionSurroundingVehicles(*localVehichIterator);

			if (1)
			{
				if (Simulator1::VehicleDrivingPlan.count(localVehichIterator->Id) == 0 || Simulator1::VehicleDrivingPlan[localVehichIterator->Id].GetCount() == 0)
				{
					if (localVehichIterator->RoadBlockType == 1)
					{
						Simulator1::MyTrajectoryPlanning.SpeedPlanningOnStraightLane(localVehichIterator->Speed, localVehichIterator->PoseAngle, localPerceptionSurroundingVehiclesList, *localVehichIterator, 0);
					}
					else
					{
						if (localVehichIterator->RoadBlockType == 2)
						{
							Simulator1::MyTrajectoryPlanning.SpeedPlanningOnIntersection(localVehichIterator->Speed, localVehichIterator->PoseAngle, localPerceptionSurroundingVehiclesList, *localVehichIterator);
							localVehichIterator->TimeCostToTargetNede = localVehichIterator->GetDistance(localVehichIterator->TargetNode) / localVehichIterator->MaxSteerSpeed;
						}
					}
				}
				else
				{
					Simulator1::VehicleDrivingPlan[localVehichIterator->Id].Run(localVehichIterator->Speed, localVehichIterator->AngularSpeed);
					localVehichIterator->PoseAngle += localVehichIterator->AngularSpeed * Simulator1::TimeStep;
				}

				localVehichIterator->UpdateVehicleLocation(Simulator1::TimeStep);
				localVehichIterator->UpdateVehicleRectangularBox();

				Simulator1::UpdateVehicle2MapBlock(*localVehichIterator);
			}
			else
			{
				localVehichIterator->Type = 4;
			}

			Simulator1::MyIntersectionStrategyEvaluation.RunOneStep(*localVehichIterator);

			localWritingIn << Simulator1::SimulatingStep << "\t" << localVehichIterator->Id << "\t" << localVehichIterator->Location.X << "\t" << localVehichIterator->Location.Y << "\t" << localVehichIterator->PoseAngle + localVehichIterator->Direction << "\t" << localVehichIterator->Speed << "\n";
		}

		for (localVehichIterator = Simulator1::VehicleOverList.begin(); localVehichIterator != Simulator1::VehicleOverList.end(); localVehichIterator++)
		{
			Simulator1::RemoveVehicle(*localVehichIterator);

			map<int, DrivingPlan>::iterator VMapIter = Simulator1::VehicleDrivingPlan.find(localVehichIterator->Id);
			if (VMapIter != Simulator1::VehicleDrivingPlan.end())
			{
				Simulator1::VehicleDrivingPlan.erase(VMapIter);
			}
		}

		if (Simulator1::SimulatingStep > 25 * 60 * 10)
		{
			break;
		}

	}
	localWritingIn.close();

}
