#pragma once

#include<map>
#include<list>
#include<vector>
#include "../../Object/Vehicle/Vehicle.h"

using namespace std;

class GroupMethod {
public:
	GroupMethod();
	~GroupMethod();
	GroupMethod(map<int, Vehicle> aSimuVehicleDict, list<int> aOriginSeqList[2], double aTimeNow);

	const double delta_1 = 1.5;
	const double delta_2 = 2;
	const int max_group_num = 12;

	const double threshold = 1.5; 

	list<int> OriginSeqList[2]; 

	map<int, Vehicle> SimuVehicleDict;
	double TimeNow;
	int* GroupBestPassingOrder;
	list<int> BestPassingOrder;

	list<list<int>> group1;
	list<list<int>> group2;
	int group1Num;
	int group2Num;

	vector<vector<int>> AllEnumSet;
	vector<int> AEnumSet;

	void Run();
	int GroupVehicles(double threshold, int index, list<list<int>>* group);
	void CalGroupMinTime(list<list<int>> group, double* groupMinTime);
	double CalGroupPassTime(double& delay, int num, double time, double minTime, int lane, int preIndex);
	void CalBestGroupPassingOrder(double* group1MinTime, double* group2MinTime);
	void EnumGroupPassingOrder(int offset, int k, int* array);
	void Convert2VehicleOrder();
};