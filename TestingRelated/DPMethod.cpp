#include "DPMethod.h"
#include "ArrivalTime.h"
#include "stdio.h"

DPMethod::DPMethod()
{

}

DPMethod::~DPMethod()
{
	DPMethod::SimuVehicleDict.clear();
	for (int i = 0; i < 2; i++) {
		DPMethod::OriginSeqList[i].clear();
	}
	DPMethod::stateSpace.clear();
	DPMethod::BestPassingOrder.clear();
}

DPMethod::DPMethod(map<int, Vehicle> aSimuVehicleDict, list<int> aOriginSeqList[2], double aTimeNow)
{
	DPMethod::SimuVehicleDict = *new map<int, Vehicle>();
	for (map<int, Vehicle>::iterator VIter = aSimuVehicleDict.begin(); VIter != aSimuVehicleDict.end(); VIter++) {
		DPMethod::SimuVehicleDict[VIter->first] = VIter->second;
	}

	for (int i = 0; i < 2; i++) {
		DPMethod::OriginSeqList[i].clear();
		for (list<int>::iterator IntIter = aOriginSeqList[i].begin(); IntIter != aOriginSeqList[i].end(); IntIter++) {
			DPMethod::OriginSeqList[i].push_back(*IntIter);
		}

		DPMethod::TimeNow = aTimeNow;
	}
}

void DPMethod::Run()
{
	int pre_mi, pre_ni, pre_ri;
	double pre_time, time;
	double pre_delay, delay;
	int pre_parent;
	State aState;
	vector<State> stateList;
	
	aState.value[0] = 0;
	aState.value[1] = 0;
	aState.value[2] = -1;
	aState.time = 0;
	aState.delay = 0;
	aState.parent = -1;
	stateList.push_back(aState);
	stateSpace.push_back(stateList);

	
	for (int i = 0; i < OriginSeqList[0].size() + OriginSeqList[1].size() ; i++) {
		stateList.clear();
		for (int j = 0; j < stateSpace[i].size(); j++) {
			GetState(stateSpace[i][j], &pre_mi, &pre_ni, &pre_ri, &pre_time, &pre_delay, &pre_parent);

			int flag;
			if (pre_mi < OriginSeqList[0].size()) {
				aState.value[0] = pre_mi + 1;
				aState.value[1] = pre_ni;
				aState.value[2] = 0;
				flag = CheckIfAppeared(stateList, aState.value);

				CalNextVehTime(pre_time, pre_delay, 0, pre_ri, GetVehMinTime(0, pre_mi + 1), &time, &delay);
				if (flag == -1) {
					aState.time = time;
					aState.delay = delay;
					aState.parent = j;
					stateList.push_back(aState);
				}
				else {
					if (time + delay < stateList[flag].time + stateList[flag].delay) {
						stateList[flag].time = time;
						stateList[flag].delay = delay;
						stateList[flag].parent = j;
					}
				}
			}

			if (pre_ni < OriginSeqList[1].size()) {
				aState.value[0] = pre_mi;
				aState.value[1] = pre_ni + 1;
				aState.value[2] = 1;
				flag = CheckIfAppeared(stateList, aState.value);

				CalNextVehTime(pre_time, pre_delay, 1, pre_ri, GetVehMinTime(1, pre_ni + 1), &time, &delay);
				if (flag == -1) {
					aState.time = time;
					aState.delay = delay;
					aState.parent = j;
					stateList.push_back(aState);
				}
				else {
					if (time + delay < stateList[flag].time + stateList[flag].delay) {
						stateList[flag].time = time;
						stateList[flag].delay = delay;
						stateList[flag].parent = j;
					}
				}
			}			
		}
		stateSpace.push_back(stateList);
	}

	stateList = stateSpace.back();
	int index;
	if (stateList.size() != 2) {
		index = 0;
	}
	else {
		index = stateList[0].time + stateList[0].delay < stateList[1].time + stateList[1].delay ? 0 : 1;
	}
	int mi, ni, ri, parent;
	GetState(stateList[index], &mi, &ni, &ri, &time, &delay, &parent);
	list<int>::reverse_iterator seqList0 = OriginSeqList[0].rbegin();
	list<int>::reverse_iterator seqList1 = OriginSeqList[1].rbegin();
	for (int i = int(OriginSeqList[0].size()) + int(OriginSeqList[1].size()) - 1; i >= 0; i--) {
		if (ri == 0) {
			BestPassingOrder.push_front(*seqList0);
			seqList0++;
		}
		else {
			BestPassingOrder.push_front(*seqList1);
			seqList1++;
		}
		GetState(stateSpace[i][parent], &mi, &ni, &ri, &time, &delay, &parent);
	}
}

void DPMethod::GetState(State aState, int* m, int* n, int* r, double* time, double* delay, int* parent) 
{
	*m = aState.value[0];
	*n = aState.value[1];
	*r = aState.value[2];
	*time = aState.time;
	*delay = aState.delay;
	*parent = aState.parent;
}

int DPMethod::CheckIfAppeared(vector<State> stateList, int aValue[])
{
	for (int i = 0; i < stateList.size(); i++)
	{
		int count = 0;
		for (int j = 0; j < 3; j++) {
			if (stateList[i].value[j] == *(aValue + j)) {
				count = count + 1;
			}
			else {
				break;
			}
		}
		if (count == 3) {
			return i;
		}
	}
	return -1;
}

void DPMethod::CalNextVehTime(double pre_time, double pre_delay, int lane, int pre_lane, double t_min, double* time, double* delay)
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

double DPMethod::GetVehMinTime(int lane, int index) 
{
	int VIter = *(next(OriginSeqList[lane].begin(), index - 1));
	ArrivalTime* MyArrivalTime = new ArrivalTime();
	double time = MyArrivalTime->CalculMinimumArrivalTimeAtOnRamp(SimuVehicleDict[VIter], TimeNow);
	delete MyArrivalTime;
	return time;
}
