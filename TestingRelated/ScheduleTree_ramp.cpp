#include "ScheduleTree_ramp.h"
#include "ScheduleTreeParameters_ramp.h"

using namespace std;

ScheduleTree_ramp::ScheduleTree_ramp()
{
	ScheduleTree_ramp::LaneWidth = paraRampLaneWidthST;
}

ScheduleTree_ramp::~ScheduleTree_ramp()
{
	delete ScheduleTree_ramp::MyCollisionAvoidSequencePlanner;
	ScheduleTree_ramp::AllPerList.clear();
	ScheduleTree_ramp::AllSeqList.clear();
	ScheduleTree_ramp::OptimizeSeqList.clear();
	ScheduleTree_ramp::SimuVehicleDict.clear();

	for (int i = 0; i < 2; i++) {
		ScheduleTree_ramp::OriginSeqList[i].clear();
	}
}

ScheduleTree_ramp::ScheduleTree_ramp(
	list<Vehicle> aDrivingInVehiclesListArray[2],
	BPointCoordinate aConflictBoundaryPointOnMainRoad,
	BPointCoordinate aConflictBoundaryPointOnRamp,
	map<int, Vehicle> aSimuVehicleDict,
	double aTimeNow)
{
	ScheduleTree_ramp::LaneWidth = paraRampLaneWidthST;

	for (int i = 0; i < 2; i++) {
		ScheduleTree_ramp::DrivingInVehiclesListArray[i].clear();
		if (aDrivingInVehiclesListArray[i].size() == 0) {
			continue;
		}
		for (list<Vehicle>::iterator VIter = aDrivingInVehiclesListArray[i].begin(); VIter != aDrivingInVehiclesListArray[i].end(); VIter++) {
			ScheduleTree_ramp::DrivingInVehiclesListArray[i].push_back(*VIter);
		}
	}
	ScheduleTree_ramp::ConflictBoundaryPointOnMainRoad = aConflictBoundaryPointOnMainRoad;
	ScheduleTree_ramp::ConflictBoundaryPointOnRamp = aConflictBoundaryPointOnRamp;

	ScheduleTree_ramp::SimuVehicleDict = *new map<int, Vehicle>();
	for (map<int, Vehicle>::iterator VIter = aSimuVehicleDict.begin(); VIter != aSimuVehicleDict.end(); VIter++) {
		ScheduleTree_ramp::SimuVehicleDict[VIter->first] = VIter->second;
	}
	
	ScheduleTree_ramp::TimeNow = aTimeNow;
}

void ScheduleTree_ramp::Init()
{
	ScheduleTree_ramp::MyCollisionAvoidSequencePlanner = new CollisionAvoidSequencePlanner();
	ScheduleTree_ramp::MyCollisionAvoidSequencePlanner->Init();
}

void ScheduleTree_ramp::Load()
{
}

void ScheduleTree_ramp::Run()
{
	ScheduleTree_ramp::GenerateOriginSeqList();

	ScheduleTree_ramp::FindDistanceFIFOPassingOrder();
	ScheduleTree_ramp::FindTimeFIFOPassingOrder();
	ScheduleTree_ramp::FindDPPassingOrder();
	ScheduleTree_ramp::FindGroupingPassingOrder();
	ScheduleTree_ramp::FindRulePassingOrder();

	ScheduleTree_ramp::AppliedPassingOrder.clear();

	cout << "\t\t\t Applied FIFO based passing order" << endl;
	for (list<int>::iterator IntIter = ScheduleTree_ramp::TimeFIFOPassingOrder.begin(); IntIter != ScheduleTree_ramp::TimeFIFOPassingOrder.end(); IntIter++)
	{
		ScheduleTree_ramp::AppliedPassingOrder.push_back(*IntIter);
	}


}

void ScheduleTree_ramp::GenerateOriginSeqList()
{
	ArrivalTime* MyArrivalTime = new ArrivalTime();

	for (int i = 0; i < 2; i++) {
		ScheduleTree_ramp::OriginSeqList[i].clear();
	}

	for (int i = 0; i < 2; i++) {
		for (list<Vehicle>::iterator localVIter = ScheduleTree_ramp::DrivingInVehiclesListArray[i].begin(); localVIter != ScheduleTree_ramp::DrivingInVehiclesListArray[i].end(); localVIter++) {
			if (localVIter->InIntersectionRadius == VehicleInIntersectionRadius::Middle) {
				if (localVIter->BlockId == 4) { 
					ScheduleTree_ramp::OriginSeqList[1].push_back(localVIter->Id);
				}
				else { 
					ScheduleTree_ramp::OriginSeqList[0].push_back(localVIter->Id);
				}

				double time = MyArrivalTime->CalculMinimumArrivalTimeAtOnRamp(*localVIter, TimeNow);
				
			}
		}
	}

	delete MyArrivalTime;
}

double ScheduleTree_ramp::Distance2RampConflictCenter(Vehicle aVehicle)
{
	double localDistance;
	if (aVehicle.BlockId == 4) {
		localDistance = sqrt((aVehicle.Location.X - ScheduleTree_ramp::ConflictBoundaryPointOnRamp.X) * (aVehicle.Location.X - ScheduleTree_ramp::ConflictBoundaryPointOnRamp.X) + (aVehicle.Location.Y - ScheduleTree_ramp::ConflictBoundaryPointOnRamp.Y) * (aVehicle.Location.Y - ScheduleTree_ramp::ConflictBoundaryPointOnRamp.Y));
	}
	else {
		localDistance = sqrt((aVehicle.Location.X - ScheduleTree_ramp::ConflictBoundaryPointOnMainRoad.X) * (aVehicle.Location.X - ScheduleTree_ramp::ConflictBoundaryPointOnMainRoad.X) + (aVehicle.Location.Y - ScheduleTree_ramp::ConflictBoundaryPointOnMainRoad.Y) * (aVehicle.Location.Y - ScheduleTree_ramp::ConflictBoundaryPointOnMainRoad.Y));
	}
	return localDistance;
}

void ScheduleTree_ramp::FindDistanceFIFOPassingOrder()
{
	map<int, double> VehicleIdMapDistance2RampConflictCenter;
	for (int i = 0; i < 2; i++) {
		for (list<int>::iterator VIdIter = ScheduleTree_ramp::OriginSeqList[i].begin(); VIdIter != ScheduleTree_ramp::OriginSeqList[i].end(); VIdIter++) {
			Vehicle localVehicle = ScheduleTree_ramp::SimuVehicleDict[*VIdIter];
			double localDistance = ScheduleTree_ramp::Distance2RampConflictCenter(localVehicle);

			VehicleIdMapDistance2RampConflictCenter.insert(make_pair(*VIdIter, localDistance));
		}
	}

	vector<pair<int, double>> tVector(VehicleIdMapDistance2RampConflictCenter.begin(), VehicleIdMapDistance2RampConflictCenter.end());
	sort(tVector.begin(), tVector.end(), local_cmp);

	ScheduleTree_ramp::DistanceFIFOPassingOrder.clear();
	for (int i = 0; i < tVector.size(); i++) {
		ScheduleTree_ramp::DistanceFIFOPassingOrder.push_back(tVector[i].first);
	}
}

void ScheduleTree_ramp::FindTimeFIFOPassingOrder()
{
	map<int, double> VehicleIdMapTimeEnterRampControlZone;
	for (int i = 0; i < 2; i++) {
		for (list<int>::iterator VIdIter = ScheduleTree_ramp::OriginSeqList[i].begin(); VIdIter != ScheduleTree_ramp::OriginSeqList[i].end(); VIdIter++) {
			Vehicle localVehicle = ScheduleTree_ramp::SimuVehicleDict[*VIdIter];
			VehicleIdMapTimeEnterRampControlZone.insert(make_pair(*VIdIter, localVehicle.TimeOfEnteringIntersectionCircleControlZone));
		}
	}

	vector<pair<int, double>> tVector(VehicleIdMapTimeEnterRampControlZone.begin(), VehicleIdMapTimeEnterRampControlZone.end());
	sort(tVector.begin(), tVector.end(), local_cmp);

	ScheduleTree_ramp::TimeFIFOPassingOrder.clear();
	for (int i = 0; i < tVector.size(); i++) {
		ScheduleTree_ramp::TimeFIFOPassingOrder.push_back(tVector[i].first);
	}
}

void ScheduleTree_ramp::FindGroupingPassingOrder()
{
	ScheduleTree_ramp::MyGroupMethod = new GroupMethod(ScheduleTree_ramp::SimuVehicleDict, ScheduleTree_ramp::OriginSeqList, ScheduleTree_ramp::TimeNow);
	MyGroupMethod->Run();

	for (list<int>::iterator OrderIter = MyGroupMethod->BestPassingOrder.begin(); OrderIter != MyGroupMethod->BestPassingOrder.end(); OrderIter++) {
		ScheduleTree_ramp::GroupPassingOrder.push_back(*OrderIter);
	}
	delete MyGroupMethod;
}

void ScheduleTree_ramp::FindDPPassingOrder()
{
	ScheduleTree_ramp::MyDPMethod = new DPMethod(ScheduleTree_ramp::SimuVehicleDict, ScheduleTree_ramp::OriginSeqList, ScheduleTree_ramp::TimeNow);
	MyDPMethod->Run();

	for (list<int>::iterator OrderIter = MyDPMethod->BestPassingOrder.begin(); OrderIter != MyDPMethod->BestPassingOrder.end(); OrderIter++) {
		ScheduleTree_ramp::DPPassingOrder.push_back(*OrderIter);
	}
	delete MyDPMethod;
}

void ScheduleTree_ramp::FindRulePassingOrder()
{
	ScheduleTree_ramp::MyRuleMethod = new RuleMethod(ScheduleTree_ramp::SimuVehicleDict, ScheduleTree_ramp::OriginSeqList, ScheduleTree_ramp::TimeNow);
	MyRuleMethod->Run();

	int ruleOrder = 1;
	for (list<int>::iterator OrderIter = MyRuleMethod->BestPassingOrder.begin(); OrderIter != MyRuleMethod->BestPassingOrder.end(); OrderIter++, ruleOrder++) {
		ScheduleTree_ramp::RulePassingOrder.push_back(*OrderIter);
		ScheduleTree_ramp::SimuVehicleDict[*OrderIter].rulePassingOrder = ruleOrder;
	}
	delete MyRuleMethod;
}

void ScheduleTree_ramp::AddToAllSeqList(list<list<list<int>>>& aAllSeqList, list<int>& aAppliedPassingOrder)
{
	aAllSeqList.clear();
	aAllSeqList = ScheduleTree_ramp::AllSeqList;

	aAppliedPassingOrder.clear();
	aAppliedPassingOrder = ScheduleTree_ramp::AppliedPassingOrder;
}

int ScheduleTree_ramp::GetAllPerListCount()
{
	return int(ScheduleTree_ramp::AllPerList.size());
}