#pragma once

#include<chrono>
#include<map>
#include"Node.h"
#include "../../Object/Vehicle/Vehicle.h"
#include"../Algorithm/Centralized/RandomNumber.h"

using namespace std::chrono;


class MCTS
{

public:
	const int DEFAULT_TIME = 2000;
	const int DEFAULT_MIN_ITERATIONS = 0;

	static constexpr float DEFAULT_C = 0.5;
	static constexpr double DEFAULT_Weight = 1;

	static constexpr float DEFAULT_W = 0.0;
	const int DEFAULT_MIN_T = 0;

	const int DEFAULT_MIN_VISITS = 0;
	int paraLaneNum6 = 2; //默认车道数

	Node* root;

	milliseconds time;
	milliseconds timeMax;

	int minIterations;

	float C;

	double weight;

	float W;

	int minT;

	int minVisits;

	unsigned int currentNodeID;

	int rule;

	microseconds selectTime, expandTime, simulateTime;
	long iterations;

public:
	vector<Vehicle> vehicleSet;
	vector<int> vehicleNoLane[40]; 
	int vehNum;
	int updataPolicyFlag;

	double bestValue;
	double FIFOvalue;
	list<int> FIFOOrder;
	vector<int> bestOrder;

	map<int, Vehicle> SimuVehicleDict;
	double TimeNow;

	MCTS(vector<Vehicle> aVehicleSet, list<int> aFIFOPassingOrder, double aConflictSubzonesLastPassedVehicleArrivalTime[20][20], map<int, Vehicle> aSimuVehicleDictOrigin, double aTimeNow) : root(new Node(0, 0, 0)), timeMax(milliseconds(DEFAULT_TIME)), weight(DEFAULT_Weight),
		time(milliseconds(DEFAULT_TIME)), minIterations(DEFAULT_MIN_ITERATIONS), C(DEFAULT_C),
		W(DEFAULT_W), minT(DEFAULT_MIN_T), minVisits(DEFAULT_MIN_VISITS), currentNodeID(0),
		selectTime(microseconds::zero()), expandTime(microseconds::zero()),
		simulateTime(microseconds::zero()), iterations(0)
	{
		MCTS::SimuVehicleDict = aSimuVehicleDictOrigin;

		MCTS::TimeNow = aTimeNow;

		vehicleSet.assign(aVehicleSet.begin(), aVehicleSet.end());
		vehNum = int(vehicleSet.size());
		for (int i = 0; i < MCTS::vehNum; i++)
		{
			for (int j = 1; j <= 4 * MCTS::paraLaneNum6; j++)
			{
				if (((vehicleSet[i].BlockId - 1)/2) * MCTS::paraLaneNum6 + vehicleSet[i].LaneId == j)
				{
					vehicleNoLane[j - 1].push_back(vehicleSet[i].Id);
					break;
				}
			}
		}

		int minID;

		for (int i = 0; i < 4 * MCTS::paraLaneNum6; i++)
		{
			for (int j = 0; j < vehicleNoLane[i].size(); j++)
			{
				minID = j;
				for (int k = j + 1; k < vehicleNoLane[i].size(); k++)
				{
					if (MCTS::SimuVehicleDict[vehicleNoLane[i][k]].LeftLaneDistance < MCTS::SimuVehicleDict[vehicleNoLane[i][minID]].LeftLaneDistance)
						minID = k;
				}
				int temp = vehicleNoLane[i][j];
				vehicleNoLane[i][j] = vehicleNoLane[i][minID];
				vehicleNoLane[i][minID] = temp;
			}
		}

		

		for (int i = 0; i < 4 * MCTS::paraLaneNum6; i++)
		{
			for (int j = 0; j < 4 * MCTS::paraLaneNum6; j++)
			{
				MCTS::ConflictSubzonesLastPassedVehicleArrivalTime[i][j] = aConflictSubzonesLastPassedVehicleArrivalTime[i][j];
			}
		}


		bestValue = (std::numeric_limits<double>::max)();
		for (list<int>::iterator IntIter = aFIFOPassingOrder.begin(); IntIter != aFIFOPassingOrder.end(); IntIter++)
		{
			MCTS::FIFOOrder.push_back(*IntIter);
		}
		MCTS::FIFOvalue = MCTS::ArrivalTimeCalculByIterativeSolution(MCTS::FIFOOrder, MCTS::SimuVehicleDict, MCTS::TimeNow);
		MCTS::updateRoot();
		updataPolicyFlag = 1;
		rule = 0;
	}

	void updateRoot(vector<double>* tAssignMin);
	void updateRoot();

	void setTime(int time) {
		this->time = milliseconds(time);
	}

	void setTimeMax(int time) {
		this->timeMax = milliseconds(time);
	}

	void setC(float C) {
		this->C = C;
	}

	void setRule(int rule) {
		this->rule = rule;
	}

	void setWeight(float weight) {
		this->weight = weight;
	}

	void setW(float W) {
		this->W = W;
	}

	void setMinT(float minT) {
		this->minT = int(minT);
	}

	void setMinVisits(int minVisits) {
		this->minVisits = minVisits;
	}

	Node* getRoot() {
		return root;
	}

	~MCTS()
	{
		delete root;
	}

	void search();

	void searchLimitedTime();

	Node* select(Node* node);

	Node* expandNext(Node* node, int action);

	void simulate(Node* node);

	void backProp(Node* node, double score);

	double ConflictSubzonesLastPassedVehicleArrivalTime[20][20];

	double ArrivalTimeCalculByIterativeSolution(list<int> aPassingOrderList, map<int, Vehicle> aSimuVehicleDict, double aTimeNow);

	double ArrivalTimeCalculByIterativeSolution(vector<int> aPassingOrderList, map<int, Vehicle> aSimuVehicleDict, double aTimeNow);

	double ObjectiveValueUpdateBasedOnNodeInformation(vector<Vehicle>& vehicleSet, vector<int> order, int depth, vector <double>* tempTAssignMin1, double* maxTAssignForEachZone1, int* maxActionIndexForEachZone1);
};
