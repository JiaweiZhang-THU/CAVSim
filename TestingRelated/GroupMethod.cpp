#include "GroupMethod.h"
#include "ArrivalTime.h"
#include "stdio.h"

#include <algorithm>

GroupMethod::GroupMethod()
{

}

GroupMethod::~GroupMethod()
{
	GroupMethod::SimuVehicleDict.clear();
	for (int i = 0; i < 2; i++) {
		GroupMethod::OriginSeqList[i].clear();
	}
	GroupMethod::group1.clear();
	GroupMethod::group2.clear();
	GroupMethod::AEnumSet.clear();
	GroupMethod::AllEnumSet.clear();
	delete GroupMethod::GroupBestPassingOrder;
	GroupMethod::BestPassingOrder.clear();
}

GroupMethod::GroupMethod(map<int, Vehicle> aSimuVehicleDict, list<int> aOriginSeqList[2], double aTimeNow)
{
	GroupMethod::SimuVehicleDict = *new map<int, Vehicle>();
	for (map<int, Vehicle>::iterator VIter = aSimuVehicleDict.begin(); VIter != aSimuVehicleDict.end(); VIter++) {
		GroupMethod::SimuVehicleDict[VIter->first] = VIter->second;
	}

	for (int i = 0; i < 2; i++) {
		GroupMethod::OriginSeqList[i].clear();
		for (list<int>::iterator IntIter = aOriginSeqList[i].begin(); IntIter != aOriginSeqList[i].end(); IntIter++) {
			GroupMethod::OriginSeqList[i].push_back(*IntIter);
		}
	}
	GroupMethod::TimeNow = aTimeNow;
}

void GroupMethod::Run()
{
	double threshold = GroupMethod::threshold;
	group1Num = GroupMethod::GroupVehicles(threshold, 0, &group1);
	group2Num = GroupMethod::GroupVehicles(threshold, 1, &group2);
	while (group1Num + group2Num > max_group_num) {
		threshold = threshold + 0.1;
		group1Num = GroupMethod::GroupVehicles(threshold, 0, &group1);
		group2Num = GroupMethod::GroupVehicles(threshold, 1, &group2);
	}
	double* group_t_min1 = new double[group1Num];
	double* group_t_min2 = new double[group2Num];
	GroupMethod::CalGroupMinTime(group1, group_t_min1);
	GroupMethod::CalGroupMinTime(group2, group_t_min2);
    GroupBestPassingOrder = new int[group1Num + group2Num];
	GroupMethod::CalBestGroupPassingOrder(group_t_min1, group_t_min2);
	GroupMethod::Convert2VehicleOrder();

	delete[] group_t_min1;
	delete[] group_t_min2;
}

int GroupMethod::GroupVehicles(double threshold, int index, list<list<int>>* group)
{
	(*group).clear();
	int num = 0;
	list<int> aGroup;
	list<list<int>>::iterator GroupIter = (*group).begin();
	list<int>::iterator lastVIdIter;
	for (list<int>::iterator VIdIter = GroupMethod::OriginSeqList[index].begin(); VIdIter != GroupMethod::OriginSeqList[index].end(); VIdIter++) {		
		if (VIdIter == GroupMethod::OriginSeqList[index].begin()) { 
			num = num + 1;
			aGroup.push_back(*VIdIter);
		}
		else {
			double headway = (GroupMethod::SimuVehicleDict[*VIdIter].LeftLaneDistance - GroupMethod::SimuVehicleDict[*lastVIdIter].LeftLaneDistance) / GroupMethod::SimuVehicleDict[*VIdIter].Speed;
			if (headway < threshold) {
				aGroup.push_back(*VIdIter);
			}
			else {
				(*group).push_back(aGroup);
				num = num + 1;
				
				aGroup.clear();
				aGroup.push_back(*VIdIter);
			}
		}
		lastVIdIter = VIdIter;
	}
	if (!aGroup.empty())
	{
		(*group).push_back(aGroup);
	}

	return num;
}

void GroupMethod::CalGroupMinTime(list<list<int>> group, double* groupMinTime)
{
	ArrivalTime* MyArrivalTime = new ArrivalTime();
	int i = 0;
	for (list<list<int>>::iterator GroupIter = group.begin(); GroupIter != group.end(); GroupIter++, i++){
		list<int>::iterator VehicleIter = (*GroupIter).begin();
		groupMinTime[i] = MyArrivalTime->CalculMinimumArrivalTimeAtOnRamp(GroupMethod::SimuVehicleDict[*VehicleIter], GroupMethod::TimeNow);
	}

	delete MyArrivalTime;
}

void GroupMethod::CalBestGroupPassingOrder(double* group1MinTime, double* group2MinTime)
{
	double least_time_delay = DBL_MAX;
	int* passingOrder = new int[group1Num + group2Num];
	int* array = new int[group1Num + group2Num];
	for (int i = 0; i < group1Num + group2Num; i++) {
		array[i] = i;
	}
	GroupMethod::EnumGroupPassingOrder(0, group1Num, array);

	for (int k = 0; k < AllEnumSet.size(); k++)
	{
		int* order = new int[group1Num + group2Num]();
		for (int i = 0; i < group1Num; i++) {
			order[AllEnumSet[k][i]] = i + 1;
		}
		
		int rampIndex = 1;
		for (int i = 0; i < group1Num + group2Num; i++) {
			if (order[i] == 0) {
				passingOrder[i] = rampIndex + group1Num;
				rampIndex++;
			}
			else {
				passingOrder[i] = order[i];
			}
		}
		double totalTime = 0;
		double totalDelay = 0;
		list<list<int>>::iterator group1Iter = group1.begin();
		list<list<int>>::iterator group2Iter = group2.begin();
		if (passingOrder[0] <= group1Num) {
			totalTime = group1MinTime[0] + ((*group1Iter).size() - 1) * delta_1;
			group1Iter++;
		}
		else {
			totalTime = group2MinTime[0] + ((*group2Iter).size() - 1) * delta_1;
			group2Iter++;
		}
		for (int i = 1; i < group1Num + group2Num; i++) {
			if (passingOrder[i] <= group1Num) {
				totalTime = CalGroupPassTime(totalDelay, int((*group1Iter).size()), totalTime, group1MinTime[passingOrder[i] - 1], 0, passingOrder[i - 1]);
				group1Iter++;
			}
			else {
				totalTime = CalGroupPassTime(totalDelay, int((*group2Iter).size()), totalTime, group2MinTime[passingOrder[i] - group1Num - 1], 1, passingOrder[i - 1]);
				group2Iter++;
			}
		}
		if (totalTime + totalDelay < least_time_delay) {
			least_time_delay = totalTime + totalDelay;
			for (int i = 0; i < group1Num + group2Num; i++) {
				*(GroupBestPassingOrder + i) = *(passingOrder + i);
			}			
		}
	}
	delete[] passingOrder;
	delete[] array;
}

void GroupMethod::EnumGroupPassingOrder(int offset, int k, int* array)
{
	if (k == 0) {
		AllEnumSet.push_back(AEnumSet);
		return;
	}
	for (int i = offset; i < group1Num + group2Num; i++) {
		AEnumSet.push_back(array[i]);
		GroupMethod::EnumGroupPassingOrder(i + 1, k - 1, array);
		AEnumSet.pop_back();
	}
}

double GroupMethod::CalGroupPassTime(double& delay, int num, double time, double minTime, int lane, int preIndex)
{
	int preLane;
	if (preIndex <= group1Num) {
		preLane = 0;
	}
	else {
		preLane = 1;
	}

	double firstTime;
	if (lane == preLane) {
		firstTime = max(time + delta_1, minTime);
	}
	else {
		firstTime = max(time + delta_2, minTime);
	}
	delay = delay + (firstTime - minTime) * num;
	return firstTime + (num - 1) * delta_1;
}

void GroupMethod::Convert2VehicleOrder()
{
	list<list<int>>::iterator group1Iter = group1.begin();
	list<list<int>>::iterator group2Iter = group2.begin();
	for (int i = 0; i < group1Num + group2Num; i++) {
		if (GroupBestPassingOrder[i] <= group1Num) {
			for (list<int>::iterator vehIter = (*group1Iter).begin(); vehIter != (*group1Iter).end(); vehIter++) {
				BestPassingOrder.push_back(*vehIter);
			}
			group1Iter++;
		}
		else {
			for (list<int>::iterator vehIter = (*group2Iter).begin(); vehIter != (*group2Iter).end(); vehIter++) {
				BestPassingOrder.push_back(*vehIter);
			}
			group2Iter++;
		}
	}
}