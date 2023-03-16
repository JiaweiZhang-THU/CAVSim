#pragma once

#include<map>
#include<list>
#include<vector>
#include "../../Object/Vehicle/Vehicle.h"

using namespace std;

struct State {
	int value[3];
	double time;
	double delay;
	int parent;
};

class DPMethod {
public:
	DPMethod();
	~DPMethod();
	DPMethod(map<int, Vehicle> aSimuVehicleDict, list<int> aOriginSeqList[2], double aTimeNow);

	const double delta_1 = 1.5;
	const double delta_2 = 2;

	list<int> OriginSeqList[2];
	map<int, Vehicle> SimuVehicleDict;
	double TimeNow;
	list<int> BestPassingOrder;

	vector<vector<State>> stateSpace;

	void Run();
	void GetState(State aState, int* m, int* n, int* r, double* time, double* delay, int* parent);
	int CheckIfAppeared(vector<State> stateList, int aValue[]);
	void CalNextVehTime(double pre_time, double pre_delay, int lane, int pre_lane, double t_min, double* time, double* delay);
	double GetVehMinTime(int lane, int index);
};