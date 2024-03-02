#include "SimulatorStraightLaneForStringStability.h"
#include "SimulatorStraightLaneForStringStabilityParameters.h"
#include "..\Algorithm\Others\Tools.h"
#include <math.h>
#include <cmath>

SimulatorStraightLaneForStringStability::SimulatorStraightLaneForStringStability()
{
	SimulatorStraightLaneForStringStability::SimulatingStep = 0;
	SimulatorStraightLaneForStringStability::SimulatingStep2Time = 0;
	SimulatorStraightLaneForStringStability::TimeStep = paraTimeStepForStringStability;
	SimulatorStraightLaneForStringStability::TimeIntervalForUpdateTrafficFlowRate = paraTimeIntervalForUpdateTrafficFlowRateForStringStability;

	SimulatorStraightLaneForStringStability::LaneNum = paraDefaultLaneNumDefinedSLBPForStringStability;

	SimulatorStraightLaneForStringStability::VehileFlowPossionLambdaPerLane = paraVehileFlowPossionLambdaPerLaneForStringStability;

	SimulatorStraightLaneForStringStability::ModularizedStraightLane.Id = 1;

	SimulatorStraightLaneForStringStability::AccumulaterdVehicleNum = 0;

	SimulatorStraightLaneForStringStability::TimeStepIntervalBetweenSnapshot = 10;

	SimulatorStraightLaneForStringStability::ThresholdOfVehicleNumInSignalLanUnit = (int)50 / 5;

	SimulatorStraightLaneForStringStability::SimulatorId = int(rand() % 10000 + 70000);

	SimulatorStraightLaneForStringStability::IsNoLaneChangeForComparision = false;
}

list<int> SimulatorStraightLaneForStringStability::VehicleFlowGenerator(int aVehicleNum, int aBeginTime)
{
	list<double> TimeInterval;
	list<double>::iterator localDoubleIter;
	for (int i = 0; i < aVehicleNum; i++)
	{
		TimeInterval.push_back(SimulatorStraightLaneForStringStability::RandomExponential());
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

BNode SimulatorStraightLaneForStringStability::InitialBNodeForNewVehicle()
{
	list<BNode> localEntranceOnRoadNetwork = SimulatorStraightLaneForStringStability::ModularizedStraightLane.EnteringVehicleInitialPositonAndPoseList;
	int EntranceNum = int(localEntranceOnRoadNetwork.size());
	int RandomEntrance = SimulatorStraightLaneForStringStability::AccumulaterdVehicleNum % EntranceNum; // 0,1,2,3...
	list<BNode>::iterator localIter = localEntranceOnRoadNetwork.begin();
	for (int i = 0; i < RandomEntrance; i++)
	{
		localIter++;
	}
	return *localIter;
}

double SimulatorStraightLaneForStringStability::RandomExponential()
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
	ExponentialValue = (-1.0 / (SimulatorStraightLaneForStringStability::VehileFlowPossionLambdaPerLane * SimulatorStraightLaneForStringStability::LaneNum)) * log(1 - ExponentialValue);
	return ExponentialValue;
}

bool SimulatorStraightLaneForStringStability::CreateNewVehicle()
{
	return true;
}

void SimulatorStraightLaneForStringStability::ClearVehicle()
{
	SimulatorStraightLaneForStringStability::VehicleList.clear();
}

void SimulatorStraightLaneForStringStability::UpdateVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = SimulatorStraightLaneForStringStability::VehicleList.begin(); localVehicleIterator != SimulatorStraightLaneForStringStability::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			*localVehicleIterator = aVehicle;
		}
	}
}

void SimulatorStraightLaneForStringStability::AddVehicle(Vehicle aVehicle)
{
	SimulatorStraightLaneForStringStability::VehicleList.push_back(aVehicle);
}

void SimulatorStraightLaneForStringStability::RemoveVehicle(Vehicle aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = SimulatorStraightLaneForStringStability::VehicleList.begin(); localVehicleIterator != SimulatorStraightLaneForStringStability::VehicleList.end(); localVehicleIterator++)
	{
		if (aVehicle.Id == localVehicleIterator->Id)
		{
			SimulatorStraightLaneForStringStability::VehicleList.erase(localVehicleIterator);
			break;
		}
	}
}

void SimulatorStraightLaneForStringStability::UpdateVehicle2MapBlock(Vehicle& aVehicle)
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
				aVehicle.RoadId = SimulatorStraightLaneForStringStability::ModularizedStraightLane.Id;
				aVehicle.BlockId = aVehicle.LaneUnitPointer->BlockId;
				aVehicle.LaneId = aVehicle.LaneUnitPointer->Id;
				break;
			}
		}

		if (localFlagIsAdjacentUnitFound == false)
		{
			if (SimulatorStraightLaneForStringStability::ModularizedStraightLane.StraightRectangle.IsPointInRect(aVehicle.Location.X, aVehicle.Location.Y) == false)
			{
				aVehicle.LaneUnitPointer->RemoveVehicle(aVehicle);

				SimulatorStraightLaneForStringStability::VehicleOverList.push_back(aVehicle);
			}
		}
	}
}

void SimulatorStraightLaneForStringStability::CheckWhetherSimulationNeedToBeReset()
{
	bool localIsNeedToResetEnv = false;
	for (list<LaneUnit>::iterator localLaneUnitPointer = SimulatorStraightLaneForStringStability::ModularizedStraightLane.StraightLaneList.front().StraightMultiLanes.begin(); localLaneUnitPointer != SimulatorStraightLaneForStringStability::ModularizedStraightLane.StraightLaneList.front().StraightMultiLanes.end(); localLaneUnitPointer++)
	{
		if (localLaneUnitPointer->VehicleList.size() > SimulatorStraightLaneForStringStability::ThresholdOfVehicleNumInSignalLanUnit)
		{
			localIsNeedToResetEnv = true;
			break;
		}
	}
	if (localIsNeedToResetEnv == true)
	{
		SimulatorStraightLaneForStringStability::VehicleOverList.clear();
		for (list<Vehicle>::iterator localVehicleIter = SimulatorStraightLaneForStringStability::VehicleList.begin(); localVehicleIter != SimulatorStraightLaneForStringStability::VehicleList.end(); localVehicleIter++)
		{
			localVehicleIter->LaneUnitPointer->RemoveVehicle(*localVehicleIter);
			SimulatorStraightLaneForStringStability::VehicleOverList.push_back(*localVehicleIter);
		}
	}
}
bool SimulatorStraightLaneForStringStability::IsLaneChangingPermitted(Vehicle aVehicel, int aLaneChangeAction)
{
	if (aVehicel.LaneId > 1 && aVehicel.LaneId < SimulatorStraightLaneForStringStability::LaneNum)
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
			if (aVehicel.LaneId == SimulatorStraightLaneForStringStability::LaneNum && aLaneChangeAction == 1)
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

list<Vehicle> SimulatorStraightLaneForStringStability::PerceptionSurroundingVehicles(Vehicle aVehicle)
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


void SimulatorStraightLaneForStringStability::SpeedDisturbanceForTheLeadingVehicle_TestScenario_Decelerate()
{
	double localTimeOfStepControl = 20.0;
	if (SimulatorStraightLaneForStringStability::SimulatingStep2Time > localTimeOfStepControl-SimulatorStraightLaneForStringStability::TimeStep and SimulatorStraightLaneForStringStability::SimulatingStep2Time < localTimeOfStepControl + SimulatorStraightLaneForStringStability::TimeStep)
	{
		for (list<Vehicle>::iterator aVehicleIter = SimulatorStraightLaneForStringStability::VehicleList.begin(); aVehicleIter != SimulatorStraightLaneForStringStability::VehicleList.end(); aVehicleIter++)
		{
			if (aVehicleIter->Id == 1)
			{
				aVehicleIter->Speed = 15;
				aVehicleIter->MaxStraightSpeed = 15;
			}
		}
	}
	
}

void SimulatorStraightLaneForStringStability::SpeedDisturbanceForTheLeadingVehicle_TestScenario_Accelerate()
{
	double localTimeOfStepControl = 20.0;
	if (SimulatorStraightLaneForStringStability::SimulatingStep2Time > localTimeOfStepControl - SimulatorStraightLaneForStringStability::TimeStep and SimulatorStraightLaneForStringStability::SimulatingStep2Time < localTimeOfStepControl + SimulatorStraightLaneForStringStability::TimeStep)
	{
		for (list<Vehicle>::iterator aVehicleIter = SimulatorStraightLaneForStringStability::VehicleList.begin(); aVehicleIter != SimulatorStraightLaneForStringStability::VehicleList.end(); aVehicleIter++)
		{
			if (aVehicleIter->Id == 1)
			{
				aVehicleIter->Speed = 25;
				aVehicleIter->MaxStraightSpeed = 25;
			}
		}
	}

}

void SimulatorStraightLaneForStringStability::InitializeAPlatoonOfVehicles(int aVehicleNum)
{
	double localDeltaDistanceBetweenTwoVehicles = 0;
	localDeltaDistanceBetweenTwoVehicles = (338 + 3380 * SimulatorStraightLaneForStringStability::StringStabilityTesting_TimeHeadway_TH) / sqrt(18561);
	list<BNode>::iterator localIter = SimulatorStraightLaneForStringStability::ModularizedStraightLane.EnteringVehicleInitialPositonAndPoseList.begin();

	for (int i = aVehicleNum; i > 0; i--)
	{
		SimulatorStraightLaneForStringStability::AccumulaterdVehicleNum += 1;

		
		Vehicle NewVehicle(localIter->Location, 0);
		NewVehicle.Location.X += (aVehicleNum - i) * localDeltaDistanceBetweenTwoVehicles;

		NewVehicle.Direction = localIter->Direction;
		NewVehicle.Id = i;
		NewVehicle.RoadId = SimulatorStraightLaneForStringStability::ModularizedStraightLane.Id;
		for (list<StraightLaneBlock>::iterator localStraightLaneBlockIter = ModularizedStraightLane.StraightLaneList.begin(); localStraightLaneBlockIter != ModularizedStraightLane.StraightLaneList.end(); localStraightLaneBlockIter++)
		{
			if (localStraightLaneBlockIter->StraightRectangle.IsPointInRect(NewVehicle.Location.X, NewVehicle.Location.Y) == true)
			{
				NewVehicle.BlockId = localStraightLaneBlockIter->Id;
			}
		}
		NewVehicle.LaneId = localIter->Id;

		NewVehicle.LaneUnitPointer = SimulatorStraightLaneForStringStability::ModularizedStraightLane.FindLaneUnitPointer(NewVehicle.BlockId, NewVehicle.LaneId);

		if (i == 1)
		{
			NewVehicle.MaxStraightSpeed = 20;
		}
		else
		{
			NewVehicle.MaxStraightSpeed = 26;
		}
		
		NewVehicle.Speed = 20;
		NewVehicle.SpeedAtLastStep = NewVehicle.Speed;
		NewVehicle.SpeedOfTheFrontVehicle = NewVehicle.Speed;
		NewVehicle.SpeedSentToFollowingVehicles = NewVehicle.Speed;

		NewVehicle.MaxStraightAccel = SimulatorStraightLaneForStringStability::StringStabilityTesting_MaxAccelerationOfVehicle;

		NewVehicle.LaneUnitPointer->AddVehicle(NewVehicle);

		SimulatorStraightLaneForStringStability::VehicleList.push_back(NewVehicle);
			}
}


void SimulatorStraightLaneForStringStability::Run()
{
	SimulatorStraightLaneForStringStability::VehileFlowPossionLambdaPerLane = 801.0 / (3600 * 10);

	SimulatorStraightLaneForStringStability::MyTrajectoryPlanning.ChoosedCarFollowingModel = 1;

	SimulatorStraightLaneForStringStability::MyTrajectoryPlanning.IsNeedCollisionCheck = true;

	list<int> VehicleArrivingTimetable = SimulatorStraightLaneForStringStability::VehicleFlowGenerator(paraPerFlowVehicleNumForStringStability, 0);

	list<int>::iterator localVehicleArrivingTimePoint = VehicleArrivingTimetable.begin();

	SimulatorStraightLaneForStringStability::SimulatingStep = 0;

	SimulatorStraightLaneForStringStability::SimulatingStep2Time = 0;

	SimulatorStraightLaneForStringStability::MyTrajectoryPlanning.IsSpeedFluctuatOnStraightLane = false;

	SimulatorStraightLaneForStringStability::MyTrajectoryPlanning.TimeStep = SimulatorStraightLaneForStringStability::TimeStep;

	SimulatorStraightLaneForStringStability::StringStabilityTesting_PacketLossRatio_PLR = 0.2;

	SimulatorStraightLaneForStringStability::StringStabilityTesting_BeaconSendingFrequence_BSF = 20;

	SimulatorStraightLaneForStringStability::StringStabilityTesting_MaxAccelerationOfVehicle = 0.6;

	
	int RandomNameValue = SimulatorStraightLaneForStringStability::SimulatorId;

	ofstream localWritingIn;
	ofstream localWritingInSimulationRecord;
	localWritingIn.open("Data/StraightLane_" + to_string(SimulatorStraightLaneForStringStability::SimulatorId) + ".txt", ios::out);

	clock_t localStartTime, localEndTime;
	localStartTime = clock();

	int localDecisonNums = 0;
	SimulatorStraightLaneForStringStability::InitializeAPlatoonOfVehicles(20);

	while (true)
	{

		SimulatorStraightLaneForStringStability::SimulatingStep += 1;
		SimulatorStraightLaneForStringStability::SimulatingStep2Time += SimulatorStraightLaneForStringStability::TimeStep;

		if (SimulatorStraightLaneForStringStability::SimulatingStep % 10 == 0)
		{
			cout << "--------------------- Simulation ID:" << SimulatorStraightLaneForStringStability::SimulatorId << " --- RunStep: " << SimulatorStraightLaneForStringStability::SimulatingStep << " --- Time: " << SimulatorStraightLaneForStringStability::SimulatingStep2Time << " s ---------------------" << endl;	
		}

		list<Vehicle> localPerceptionSurroundingVehiclesList;
		list<Vehicle> localPerceptionSurroundingVehiclesListPlus;
		list<Vehicle>::iterator localVehichIterator;

		SimulatorStraightLaneForStringStability::SpeedDisturbanceForTheLeadingVehicle_TestScenario_Decelerate();

		for (SimulatorStraightLaneForStringStability::VehicleOverList.clear(), localVehichIterator = SimulatorStraightLaneForStringStability::VehicleList.begin(); localVehichIterator != SimulatorStraightLaneForStringStability::VehicleList.end(); localVehichIterator++)
		{
			localPerceptionSurroundingVehiclesList = SimulatorStraightLaneForStringStability::PerceptionSurroundingVehicles(*localVehichIterator);

			int localLaneChangeAction = 0;

			if (localLaneChangeAction != 0)
			{
				localVehichIterator->DistannceFromTargetLane = paraLaneWidthInSimulatorForStringStability;
			}
			localVehichIterator->LastLaneChangeAction = localLaneChangeAction;

			SimulatorStraightLaneForStringStability::MyTrajectoryPlanning.SpeedPlanningOnStraightLaneForStringStability(
				localVehichIterator->Speed, 
				localVehichIterator->PoseAngle, 
				localPerceptionSurroundingVehiclesList, 
				*localVehichIterator, 
				localLaneChangeAction,
				SimulatorStraightLaneForStringStability::StringStabilityTesting_PacketLossRatio_PLR,
				SimulatorStraightLaneForStringStability::StringStabilityTesting_TimeHeadway_TH
			);

			localVehichIterator->UpdateVehicleLocationForStringStability(SimulatorStraightLaneForStringStability::TimeStep);
			
			if(SimulatorStraightLaneForStringStability::SimulatingStep%int((1/ SimulatorStraightLaneForStringStability::StringStabilityTesting_BeaconSendingFrequence_BSF)/SimulatorStraightLaneForStringStability::TimeStep) == 0)
			{
				localVehichIterator->UpdateSpeedSentToFollowingVehicles();
			}

			localVehichIterator->IsCollideWithOtherVehicles(localPerceptionSurroundingVehiclesList);

			SimulatorStraightLaneForStringStability::UpdateVehicle2MapBlock(*localVehichIterator);

			if (1)
			{
				localWritingIn << SimulatorStraightLaneForStringStability::SimulatingStep 
					<< "\t" << localVehichIterator->Id 
					<< "\t" << localVehichIterator->Location.X 
					<< "\t" << localVehichIterator->Location.Y 
					<< "\t" << localVehichIterator->PoseAngle + localVehichIterator->Direction 
					<< "\t" << localVehichIterator->Speed << "\n";
			}
		}

		for (localVehichIterator = SimulatorStraightLaneForStringStability::VehicleOverList.begin(); localVehichIterator != SimulatorStraightLaneForStringStability::VehicleOverList.end(); localVehichIterator++)
		{
			SimulatorStraightLaneForStringStability::RemoveVehicle(*localVehichIterator);
		}

		int localRuningTime = 5000;
		
		if (SimulatorStraightLaneForStringStability::SimulatingStep > localRuningTime)
		{
			break;
		}
		if (SimulatorStraightLaneForStringStability::VehicleList.size() == 0)
		{
			break;
		}
	}
	localWritingIn.close();

	localEndTime = clock();

	cout << "\nThe run time is: " << (double)(localEndTime - localStartTime) / CLOCKS_PER_SEC << " s" << endl;
}
