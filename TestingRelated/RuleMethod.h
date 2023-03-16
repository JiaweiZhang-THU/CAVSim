#pragma once

#include<map>
#include<list>
#include<vector>
#include "../../Object/Vehicle/Vehicle.h"

using namespace std;

static bool local_cmp(const pair<int, double>& a, const pair<int, double>& b) {
	return a.second < b.second;
}

struct Path {
	int lane;
	int index;
	double time;
};

class RuleMethod {
public:
	RuleMethod();
	~RuleMethod();
	RuleMethod(map<int, Vehicle> aSimuVehicleDict, list<int> aOriginSeqList[2], double aTimeNow);

	const double delta_1 = 1.5;
	const double delta_2 = 2;

	list<int> OriginSeqList[2];

	map<int, Vehicle> SimuVehicleDict;
	double TimeNow;
	list<int> BestPassingOrder;

	vector<double> T_min0, T_min1; 

	vector<Path> RulePath;

	void Run();
	int GetNewPassingOrder();
	void CalNextVehTime(double pre_time, double pre_delay, int lane, int pre_lane, double t_min, double* time, double* delay);
	double GetVehMinTime(int lane, int index);
	void ReplanPath(int* array);
	double CalPathTime(vector<Path>* aRulePath);
	void EnumMethod(int* newVeh);
};