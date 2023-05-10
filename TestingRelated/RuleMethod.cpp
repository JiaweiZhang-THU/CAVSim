#include "RuleMethod.h"
#include "ArrivalTime.h"
#include "stdio.h"
#include <algorithm>

RuleMethod::RuleMethod()
{

}

RuleMethod::~RuleMethod()
{
	RuleMethod::SimuVehicleDict.clear();
	for (int i = 0; i < 2; i++) {
		RuleMethod::OriginSeqList[i].clear();
	}
	RuleMethod::BestPassingOrder.clear();
	RuleMethod::RulePath.clear();
	RuleMethod::T_min0.clear();
	RuleMethod::T_min1.clear();
}

RuleMethod::RuleMethod(map<int, Vehicle> aSimuVehicleDict, list<int> aOriginSeqList[2], double aTimeNow)
{
	RuleMethod::SimuVehicleDict = *new map<int, Vehicle>();
	for (map<int, Vehicle>::iterator VIter = aSimuVehicleDict.begin(); VIter != aSimuVehicleDict.end(); VIter++) {
		RuleMethod::SimuVehicleDict[VIter->first] = VIter->second;
	}

	for (int i = 0; i < 2; i++) {
		RuleMethod::OriginSeqList[i].clear();
		for (list<int>::iterator IntIter = aOriginSeqList[i].begin(); IntIter != aOriginSeqList[i].end(); IntIter++) {
			RuleMethod::OriginSeqList[i].push_back(*IntIter);
		}

		RuleMethod::TimeNow = aTimeNow;
	}
}

void RuleMethod::Run()
{
	int newEnterNum = GetNewPassingOrder();
	int satisfied_flag; 

	for (int i = int(RulePath.size()) - newEnterNum; i < int(RulePath.size()); i++) {
		if (i < 2) {
			int newVeh[2] = { RulePath[i].lane, i };
			EnumMethod(newVeh);
			continue;
		}

		satisfied_flag = 0;

		if (RulePath[i].lane == 0) {
			int ramp_count = 0; 

			while (i - ramp_count > 0 && RulePath[i - ramp_count - 1].lane == 1) {
				ramp_count = ramp_count + 1;
			}
			if (ramp_count == 1) {
				if (RulePath[i - 2].time + delta_1 >= T_min0[RulePath[i].index]) {
					satisfied_flag = 1;
					int array[3] = { i - 1, i , 1 };
					ReplanPath(array);
				}
			}
			else if (ramp_count >= 2) {
				int ramp_fore_index = i - ramp_count - 1;

				while (ramp_fore_index >= 0) {
					if (RulePath[ramp_fore_index].lane == 1) {
						if (RulePath[ramp_fore_index].time + delta_1 >= T_min1[RulePath[i - ramp_count].index] && RulePath[i - ramp_count].time + (ramp_count - 1) * delta_1 >= T_min1[RulePath[i - 1].index]) {
							satisfied_flag = 1;
							int array[3] = {ramp_fore_index + 1, i - 1, i - ramp_count - ramp_fore_index - 1};
							ReplanPath(array);
						}
						break;
					}
					else {
						ramp_fore_index = ramp_fore_index - 1;
					}
				}
			}			
		}
		else {
			int main_count = 0;

			while (i - main_count > 0 && RulePath[i - main_count - 1].lane == 0) {
				main_count = main_count + 1;
			}
			if (main_count == 1) {
				if (RulePath[i - 2].time + delta_1 >= T_min1[RulePath[i].index]) {
					satisfied_flag = 1;
					int array[3] = { i - 1, i, 1 };
					ReplanPath(array);
				}
			}
			else if (main_count >= 2) {
				int main_fore_index = i - main_count - 1; 

				while (main_fore_index >= 0) {
					if (RulePath[main_fore_index].lane == 0) {
						if (RulePath[main_fore_index].time + delta_1 >= T_min0[RulePath[i - main_count].index] && RulePath[i - main_count].time + (main_count - 1) * delta_1 >= T_min0[RulePath[i - 1].index]) {
							satisfied_flag = 1;
							int array[3] = { main_fore_index + 1, i - 1, i - main_count - main_fore_index - 1 };
							ReplanPath(array);
						}
						break;
					}
					else {
						main_fore_index = main_fore_index - 1;
					}
				}
			}
		}

		if (satisfied_flag == 0) {
			int newVeh[2] = { RulePath[i].lane, i };
			EnumMethod(newVeh);
		}
	}

	for (int i = 0; i < RulePath.size(); i++) {
		BestPassingOrder.push_back(*(next(OriginSeqList[RulePath[i].lane].begin(), RulePath[i].index)));
	}
}

int RuleMethod::GetNewPassingOrder()
{
	map<int, int> lastMapRulePassingOrder; 

	map<int, double> newEnterVeh;

	for (int i = 0; i < 2; i++) {
		int index = (i == 0) ? -1 : (int(OriginSeqList[0].size()) - 1);
		for (list<int>::iterator IntIter = OriginSeqList[i].begin(); IntIter != OriginSeqList[i].end(); IntIter++) {
			index = index + 1;
			if (SimuVehicleDict[*IntIter].rulePassingOrder != -1) {
				lastMapRulePassingOrder.insert(make_pair(index, SimuVehicleDict[*IntIter].rulePassingOrder));
			}
			else {

				newEnterVeh.insert(make_pair(index, SimuVehicleDict[*IntIter].TimeOfEnteringIntersectionCircleControlZone));
			}
		}
	}
	vector<pair<int, int>> tVector(lastMapRulePassingOrder.begin(), lastMapRulePassingOrder.end());
	sort(tVector.begin(), tVector.end(), local_cmp);

	vector<pair<int, double>> ttVector(newEnterVeh.begin(), newEnterVeh.end());
	sort(ttVector.begin(), ttVector.end(), local_cmp); 

	Path curPath;
	for (int i = 0; i < tVector.size() + ttVector.size(); i++) {
		int temp = (i < tVector.size()) ? tVector[i].first : ttVector[i - tVector.size()].first; 

		if (temp < OriginSeqList[0].size()) {
			curPath.lane = 0;
			curPath.index = temp;
			curPath.time = GetVehMinTime(curPath.lane, curPath.index);
			T_min0.push_back(curPath.time);
		}
		else {
			curPath.lane = 1;
			curPath.index = temp - int(OriginSeqList[0].size());
			curPath.time = GetVehMinTime(curPath.lane, curPath.index);
			T_min1.push_back(curPath.time);
		}		
		RulePath.push_back(curPath);
	}
	double delay = 0;
	for (int i = 0; i < RulePath.size(); i++) {
		CalPathTime(&RulePath);
	}

	return int(ttVector.size());
}

double RuleMethod::CalPathTime(vector<Path>* aRulePath)
{
	double delay = 0;
	if ((*aRulePath)[0].lane == 0) {
		(*aRulePath)[0].time = T_min0[0];
	}
	else {
		(*aRulePath)[0].time = T_min1[0];
	}
	for (int i = 1; i < (*aRulePath).size(); i++) {
		if ((*aRulePath)[i].lane == 0) {
			CalNextVehTime((*aRulePath)[i - 1].time, delay, 0, (*aRulePath)[i - 1].lane, T_min0[(*aRulePath)[i].index], &(*aRulePath)[i].time, &delay);
		}
		else {
			CalNextVehTime((*aRulePath)[i - 1].time, delay, 1, (*aRulePath)[i - 1].lane, T_min1[(*aRulePath)[i].index], &(*aRulePath)[i].time, &delay);
		}
	}
	return delay;
}

void RuleMethod::CalNextVehTime(double pre_time, double pre_delay, int lane, int pre_lane, double t_min, double* time, double* delay)
{
	if (pre_time == 0) {
		*time = t_min;
	}
	else {
		if (lane == pre_lane) {
			*time = max(t_min, pre_time + delta_1);
		}
		else {
			*time = max(t_min, pre_time + delta_2);
		}
	}
	*delay = pre_delay + *time - t_min;
}

double RuleMethod::GetVehMinTime(int lane, int index)
{
	int VIter = *(next(OriginSeqList[lane].begin(), index));
	ArrivalTime* MyArrivalTime = new ArrivalTime();
	double time = MyArrivalTime->CalculMinimumArrivalTimeAtOnRamp(SimuVehicleDict[VIter], TimeNow);
	delete MyArrivalTime;
	return time;
}

void RuleMethod::ReplanPath(int* array)
{

	vector<Path> newPath;
	newPath.insert(newPath.end(), RulePath.begin(), RulePath.begin() + array[0]);
	newPath.insert(newPath.end(), RulePath.begin() + array[0] + array[2], RulePath.begin() + array[1] + 1);
	newPath.insert(newPath.end(), RulePath.begin() + array[0], RulePath.begin() + array[0] + array[2]);
	newPath.insert(newPath.end(), RulePath.begin() + array[1] + 1, RulePath.end());
	RulePath = newPath;
	CalPathTime(&RulePath);
}

void RuleMethod::EnumMethod(int* newVeh)
{

	int fore_index = -1;
	for (int index = 1; index <= newVeh[1]; index++) {
		if (RulePath[newVeh[1] - index].lane == newVeh[0]) {
			fore_index = newVeh[1] - index;
			break;
		}
	}

	if (fore_index == newVeh[1] - 1) {
		return; 
	}
	else {
		double minValue = DBL_MAX;
		double delay;
		vector<Path> newPath, bestPath;
		for (int i = fore_index + 1; i <= newVeh[1]; i++) {
			newPath.assign(RulePath.begin(), RulePath.begin() + newVeh[1]);
			newPath.insert(newPath.begin() + i, RulePath[newVeh[1]]);
			delay = CalPathTime(&newPath);
			if (delay + newPath.back().time < minValue) {
				minValue = delay + newPath.back().time;
				bestPath = newPath;
			}
		}
		bestPath.insert(bestPath.end(), RulePath.begin() + newVeh[1] + 1, RulePath.end());
		RulePath = bestPath;
	}
}