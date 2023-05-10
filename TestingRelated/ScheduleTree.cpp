#include "ScheduleTree.h"
#include "ScheduleTreeParameters.h"

ScheduleTree::ScheduleTree()
{
	ScheduleTree::LaneWidth = paraLaneWidthST;

	ScheduleTree::IntersectionOneWayLanesNum = int(paraLaneNum);

	ScheduleTree::ToleranceThreshold = paraToleranceThreshold;
}

ScheduleTree::~ScheduleTree()
{
	delete ScheduleTree::MyCollisionAvoidSequencePlanner;

	ScheduleTree::AllPerList.clear();

	ScheduleTree::AllSeqList.clear();

	ScheduleTree::OptimizeSeqList.clear();
	
	ScheduleTree::SimuVehicleDict.clear();

	for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		ScheduleTree::OriginSeqList[i].clear();
	}

}

ScheduleTree::ScheduleTree(
	list<Vehicle> aDrivingInVehiclesListArray[4 * 10], 
	int aIntersectionOneWayLanesNum, 
	BPointCoordinate aIntersectionCenter, 
	map<int, Vehicle> aSimuVehicleDict, 
	double aConflictSubzonesLastPassedVehicleArrivalTime[20][20],
	double aTimeNow)
{
	ScheduleTree::LaneWidth = paraLaneWidthST;

	ScheduleTree::IntersectionOneWayLanesNum = aIntersectionOneWayLanesNum;

	ScheduleTree::ToleranceThreshold = paraToleranceThreshold;

	for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		ScheduleTree::DrivingInVehiclesListArray[i].clear();

		if (aDrivingInVehiclesListArray[i].size() == 0)
		{
			continue;
		}

		for (list<Vehicle>::iterator VIter = aDrivingInVehiclesListArray[i].begin(); VIter != aDrivingInVehiclesListArray[i].end(); VIter++)
		{
			ScheduleTree::DrivingInVehiclesListArray[i].push_back(*VIter);
		}
	}

	ScheduleTree::IntersectionCenter = aIntersectionCenter;

	ScheduleTree::SimuVehicleDict = *new map<int, Vehicle>();

	for (map<int, Vehicle>::iterator VIter = aSimuVehicleDict.begin(); VIter != aSimuVehicleDict.end(); VIter++)
	{
		ScheduleTree::SimuVehicleDict[VIter->first] = VIter->second;
	}

	for (int i = 0; i < 2 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		for (int j = 0; j < 2 * ScheduleTree::IntersectionOneWayLanesNum; j++)
		{
			ScheduleTree::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = aConflictSubzonesLastPassedVehicleArrivalTime[i][j];
		}
	}

	ScheduleTree::TimeNow = aTimeNow;
}

void ScheduleTree::Init()
{
	ScheduleTree::MyCollisionAvoidSequencePlanner = new CollisionAvoidSequencePlanner();

	ScheduleTree::MyCollisionAvoidSequencePlanner->Init();
}

void ScheduleTree::Load()
{
}

void ScheduleTree::Run()
{
	
	ScheduleTree::GenerateOriginSeqList();

	ScheduleTree::CalculatePassingIndex();

	ScheduleTree::FindDistanceFIFOPassingOrder();

	ScheduleTree::FindTimeFIFOPassingOrder();

	ScheduleTree::FindMCTSPassingOrder();

	ScheduleTree::AppliedPassingOrder.clear();

	cout << "\t\t\tApplied FIFO based Order: [";

	for (list<int>::iterator IntIter = ScheduleTree::TimeFIFOPassingOrder.begin(); IntIter != ScheduleTree::TimeFIFOPassingOrder.end(); IntIter++)
	{
		cout << *IntIter << ",";

		ScheduleTree::AppliedPassingOrder.push_back(*IntIter);
	}
	cout << "]\n" << endl;
}

void ScheduleTree::FindMCTSPassingOrder()
{
	vector<Vehicle> vechileSet;
	for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		for (list<int>::iterator VIdIter = ScheduleTree::OriginSeqList[i].begin(); VIdIter != ScheduleTree::OriginSeqList[i].end(); VIdIter++)
		{
			Vehicle localVehicle = ScheduleTree::SimuVehicleDict[*VIdIter];
			vechileSet.push_back(localVehicle);
		}
	}

	bool IsFIFOPassingOrder = paraIsFIFOPassingOrder;
	if (IsFIFOPassingOrder == false)
	{
		MCTS* MyMCTS = new MCTS(
			vechileSet,
			ScheduleTree::DistanceFIFOPassingOrder,
			ScheduleTree::ConflictSubzonesLastPassedVehicleArrivalTime,
			ScheduleTree::SimuVehicleDict,
			ScheduleTree::TimeNow);

		MyMCTS->setTimeMax(int(paraSearchtime));
		MyMCTS->setC(float(0.05));
		MyMCTS->setWeight(float(0.85));
		MyMCTS->setRule(2);

		MyMCTS->searchLimitedTime();

		ScheduleTree::MCTSPassingOrder.clear();
		for (vector<int>::iterator IntIter = MyMCTS->bestOrder.begin(); IntIter != MyMCTS->bestOrder.end(); IntIter++)
		{
			ScheduleTree::MCTSPassingOrder.push_back(*IntIter);
		}
		delete MyMCTS;
	}
	else
	{
	}
}

void ScheduleTree::GenerateOriginSeqList()
{
	for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		ScheduleTree::OriginSeqList[i].clear();
	}

	int seqListIndex = 0;
	for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		for (list<Vehicle>::iterator localVIter = ScheduleTree::DrivingInVehiclesListArray[i].begin(); localVIter != ScheduleTree::DrivingInVehiclesListArray[i].end(); localVIter++)
		{
			if (localVIter->InIntersectionRadius == VehicleInIntersectionRadius::Middle)
			{
				ScheduleTree::OriginSeqList[i].push_back(localVIter->Id);
			}
		}
	}
}

double ScheduleTree::Distance2IntersectionCenter(Vehicle aVehicle)
{
	double localDistance = sqrt((aVehicle.Location.X - ScheduleTree::IntersectionCenter.X) * (aVehicle.Location.X - ScheduleTree::IntersectionCenter.X) + (aVehicle.Location.Y - ScheduleTree::IntersectionCenter.Y) * (aVehicle.Location.Y - ScheduleTree::IntersectionCenter.Y));
	return localDistance;
}


void ScheduleTree::FindDistanceFIFOPassingOrder()
{
	ScheduleTree::AllPerList.clear();
	map<int, double> VehicleIdMapDistance2IntersectionCenter;
	for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		for (list<int>::iterator VIdIter = ScheduleTree::OriginSeqList[i].begin(); VIdIter != ScheduleTree::OriginSeqList[i].end(); VIdIter++)
		{
			Vehicle localVehicle = ScheduleTree::SimuVehicleDict[*VIdIter];
			double localDistance = ScheduleTree::Distance2IntersectionCenter(localVehicle);

			VehicleIdMapDistance2IntersectionCenter.insert(make_pair(*VIdIter, localDistance));
		}
	}

	vector<pair<int,double>> tVector(VehicleIdMapDistance2IntersectionCenter.begin(),VehicleIdMapDistance2IntersectionCenter.end());
	sort(tVector.begin(), tVector.end(),local_cmp);

	ScheduleTree::DistanceFIFOPassingOrder.clear();
	for (int i = 0; i < tVector.size(); i++)
	{
		ScheduleTree::DistanceFIFOPassingOrder.push_back(tVector[i].first);
	}

	ScheduleTree::AllPerList.push_back(ScheduleTree::DistanceFIFOPassingOrder);
}



void ScheduleTree::FindTimeFIFOPassingOrder()
{
	map<int, double> VehicleIdMapTimeEnterIntersectionControlZone;
	for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		for (list<int>::iterator VIdIter = ScheduleTree::OriginSeqList[i].begin(); VIdIter != ScheduleTree::OriginSeqList[i].end(); VIdIter++)
		{
			Vehicle localVehicle = ScheduleTree::SimuVehicleDict[*VIdIter];
			VehicleIdMapTimeEnterIntersectionControlZone.insert(make_pair(*VIdIter, localVehicle.TimeOfEnteringIntersectionCircleControlZone));
		}
	}

	vector<pair<int, double>> tVector(VehicleIdMapTimeEnterIntersectionControlZone.begin(), VehicleIdMapTimeEnterIntersectionControlZone.end());
	sort(tVector.begin(), tVector.end(), local_cmp);

	ScheduleTree::TimeFIFOPassingOrder.clear();
	for (int i = 0; i < tVector.size(); i++)
	{
		ScheduleTree::TimeFIFOPassingOrder.push_back(tVector[i].first);
	}
}


double ScheduleTree::ArrivalTimeCalculByIterativeSolution(list<int> aPassingOrderList)
{
	ArrivalTime* MyArrivalTime = new ArrivalTime(
		ScheduleTree::ConflictSubzonesLastPassedVehicleArrivalTime,
		ScheduleTree::IntersectionOneWayLanesNum
	);

	double JTotalDelay = MyArrivalTime->PassingOrderToTrajectoryInterPretaton(
		aPassingOrderList,
		ScheduleTree::SimuVehicleDict,
		ScheduleTree::TimeNow
	);

	delete MyArrivalTime;

	return JTotalDelay;
}

void ScheduleTree::InteriorEnumerationLoop(int leftCount, int sum)
{
	if (ScheduleTree::AllPerList.size() > 1000)
	{
		return;
	}
	if (leftCount == 0)
	{
		list<int> newOrder;
		for (int i = 0; i < sum; i++)
		{
			newOrder.push_back(ScheduleTree::OnePossiblePermutationOrder[i]);
		}
		ScheduleTree::AllPerList.push_back(newOrder);
	}
	else
	{
		for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
		{
			if (ScheduleTree::OriginSeqList[i].size() != 0)
			{
				int tempValue = ScheduleTree::OriginSeqList[i].front();
				if (leftCount != sum)
				{
					Vehicle former = ScheduleTree::SimuVehicleDict[ScheduleTree::OnePossiblePermutationOrder[sum - leftCount - 1]];
					Vehicle latter = ScheduleTree::SimuVehicleDict[tempValue];

					if (!ScheduleTree::isWiseToArrangeSo(former, latter))
					{
						continue;
					}
				}

				ScheduleTree::OnePossiblePermutationOrder[sum - leftCount] = tempValue;

				ScheduleTree::OriginSeqList[i].pop_front();

				ScheduleTree::InteriorEnumerationLoop(leftCount - 1, sum);

				ScheduleTree::OriginSeqList[i].push_front(tempValue);
			}
		}
	}
}

void ScheduleTree::GenerateAllSeqList()
{
	for (list<list<int>>::iterator possPerListIter = ScheduleTree::AllPerList.begin(); possPerListIter != ScheduleTree::AllPerList.end(); possPerListIter++)
	{
		int* rawSeqArray = new int[possPerListIter->size()];
		list<int> rawSeqlist = *new list<int>();

		list<int>::iterator intIter = possPerListIter->begin();
		for (int i = 0; i < possPerListIter->size(); i++, intIter++)
		{
			rawSeqArray[i] = *intIter;
			rawSeqlist.push_back(*intIter);
		}
		ScheduleTree::MyCollisionAvoidSequencePlanner->Load(rawSeqlist,ScheduleTree::AllSeqList);
		ScheduleTree::MyCollisionAvoidSequencePlanner->Run();
		
		ScheduleTree::MyCollisionAvoidSequencePlanner->AddToTheAllSeqListCite(ScheduleTree::AllSeqList);
	}
}

void ScheduleTree::AddToAllSeqList(list<list<list<int>>>& aAllSeqList, list<int>& aAppliedPassingOrder)
{
	aAllSeqList.clear();
	aAllSeqList = ScheduleTree::AllSeqList;

	aAppliedPassingOrder.clear();
	aAppliedPassingOrder = ScheduleTree::AppliedPassingOrder;
}

int ScheduleTree::GetAllPerListCount()
{
	return int(ScheduleTree::AllPerList.size());
}

void ScheduleTree::CalculatePassingIndex()
{
	for (int i = 0; i < 4 * ScheduleTree::IntersectionOneWayLanesNum; i++)
	{
		list<int> oneList = ScheduleTree::OriginSeqList[i];
		for (list<int>::iterator intIter = oneList.begin(); intIter != oneList.end(); intIter++)
		{
			Vehicle vehicle = ScheduleTree::SimuVehicleDict[*intIter];
			double distFactor = vehicle.LeftLaneDistance;
			double speedFactor = vehicle.Speed * vehicle.Speed / 2 / vehicle.MaxStraightAccel;
			ScheduleTree::SimuVehicleDict[*intIter].QueueingIndex = distFactor - speedFactor;
		}
	}
}

bool ScheduleTree::isWiseToArrangeSo(Vehicle former, Vehicle latter)
{
	if (former.QueueingIndex - latter.QueueingIndex > ScheduleTree::ToleranceThreshold)
	{
		return false;
	}
	return true;
}