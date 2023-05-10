#include "MCTS.h"
#include "ArrivalTime.h"

void MCTS::updateRoot()
{
	for (int i = 0; i < 4 * MCTS::paraLaneNum6; i++)
	{
		MCTS::root->remainingVehicleNum.push_back(MCTS::vehicleNoLane[i].size());
	}
	int i = 0;
	for (std::vector<int>::iterator it = MCTS::root->remainingVehicleNum.begin(); it != MCTS::root->remainingVehicleNum.end(); ++it)
	{
		if (*it != 0)
		{
			MCTS::root->remainingAction.push_back(i);
			MCTS::root->childNum++;
		}
		i++;
	}
	MCTS::root->childNo.clear();

	for (int i = 0; i < MCTS::paraLaneNum6 * 2 * MCTS::paraLaneNum6 * 2; i++)
	{
		MCTS::root->maxTAssignForEachZone[i] = 0;
	}
	for (int i = 0; i < MCTS::paraLaneNum6 * 2 * MCTS::paraLaneNum6 * 2; i++)
	{
		MCTS::root->maxActionIndexForEachZone[i] = -1;
	}
}

void MCTS::updateRoot(vector<double>* tAssignMin)
{
	for (int i = 0; i < 4 * MCTS::paraLaneNum6; i++)
	{
		MCTS::root->remainingVehicleNum.push_back(MCTS::vehicleNoLane[i].size());
	}
	int i = 0;
	for (std::vector<int>::iterator it = MCTS::root->remainingVehicleNum.begin(); it != MCTS::root->remainingVehicleNum.end(); ++it)
	{
		if (*it != 0)
		{
			MCTS::root->remainingAction.push_back(i);
			MCTS::root->childNum++;
		}
		i++;
	}
	MCTS::root->childNo.clear();

	for (int i = 0; i < MCTS::paraLaneNum6 *2* MCTS::paraLaneNum6 * 2; i++)
	{
		MCTS::root->tempTAssignMin[i].assign(tAssignMin[i].begin(), tAssignMin[i].end());
	}
	for (int i = 0; i < MCTS::paraLaneNum6 * 2 * MCTS::paraLaneNum6 * 2; i++)
	{
		MCTS::root->maxTAssignForEachZone[i] = 0;
	}
	for (int i = 0; i < MCTS::paraLaneNum6 * 2 * MCTS::paraLaneNum6 * 2; i++)
	{
		MCTS::root->maxActionIndexForEachZone[i] = -1;
	}
}

void MCTS::search()
{
	system_clock::time_point old = system_clock::now();

	while (duration_cast<milliseconds>(system_clock::now() - old) < time && (!root->terminalFlag))
	{
		Node* selected = root;
		while (!selected->shouldExpand())
			selected = select(selected);

		Node* expanded;
		double action = selected->remainingAction[0];
		int numVisits = selected->getNumVisits();
		if (numVisits >= minT) 
		{
			for (int i = 0; i < selected->childNo.size(); i++)
			{
				if (selected->childNo[i] != selected->remainingAction[i])
				{
					action = selected->remainingAction[i];
					break;
				}
				action = selected->remainingAction[i + 1];
			}
			expanded = expandNext(selected, int(action));
		}
		else
		{
			expanded = selected;
		}

		simulate(expanded);
		backProp(expanded, expanded->score);
	}
}

void MCTS::searchLimitedTime()
{
	system_clock::time_point old = system_clock::now();
	int localIterationNum = 0;
	while (duration_cast<milliseconds>(system_clock::now() - old) < timeMax && (!root->terminalFlag))
	{

		Node* selected = root;
		while (!selected->shouldExpand())
			selected = select(selected);

		Node* expanded;
		double action = selected->remainingAction[0];
		int numVisits = selected->getNumVisits();
		if (numVisits >= minT) 
		{
			for (int i = 0; i < selected->childNo.size(); i++)
			{
				if (selected->childNo[i] != selected->remainingAction[i])
				{
					action = selected->remainingAction[i];
					break;
				}
				action = selected->remainingAction[i + 1];
			}
			expanded = expandNext(selected, int(action));
		}
		else
		{
			expanded = selected;
		}
		simulate(expanded);

		backProp(expanded, expanded->score);

		localIterationNum++;
	}
}

Node* MCTS::select(Node* node)
{
	Node* best = nullptr;
	double bestScore = 0;

	std::vector<Node*>& children = node->getChildren();

	if (children.size() > 1)
	{
		if (rule != 3)
		{
			double maxTemp = 0;
			double minTemp = 10000;
			for (Node* n : children)
			{
				if (n->dataTemporary > maxTemp)
					maxTemp = n->dataTemporary;

				if (n->dataTemporary < minTemp)
					minTemp = n->dataTemporary;
			}

			bool equalFlag = (fabs(maxTemp - minTemp) < 0.01);

			for (Node* n : children)
			{
				if (n->terminalFlag) 
					continue;

				double score;
				if (equalFlag)
				{
					score = n->data + C * (double)sqrt(log(node->getNumVisits()) / n->getNumVisits());
				}
				else
				{
					double scoreTemp = 1 - ((n->dataTemporary - minTemp) / (maxTemp - minTemp));
					score = (1 - weight) * n->data + weight * scoreTemp + C * (double)sqrt(log(node->getNumVisits()) / n->getNumVisits());
				}

				if (score >= bestScore)
				{
					bestScore = score;
					best = n;
				}
			}
		}
		else
		{
			double maxTemp = 0;
			double minTemp = 10000;
			double maxFuture = 0;
			double minFuture = 10000;
			for (Node* n : children)
			{
				if (n->dataTemporary > maxTemp)
					maxTemp = n->dataTemporary;

				if (n->dataTemporary < minTemp)
					minTemp = n->dataTemporary;

				if (n->dataFuture > maxFuture)
					maxFuture = n->dataFuture;

				if (n->dataFuture < minFuture)
					minFuture = n->dataFuture;
			}

			bool equalFlag = (fabs(maxTemp - minTemp) < 0.01);
			bool equalFlagFuture = (fabs(maxFuture - minFuture) < 0.01);

			for (Node* n : children)
			{
				if (n->terminalFlag) 
					continue;

				double score;
				if (equalFlag && equalFlagFuture)
				{
					score = C * (double)sqrt(log(node->getNumVisits()) / n->getNumVisits());
				}
				else if (equalFlag == true && equalFlagFuture == false)
				{
					double scoreTemp = 1 - ((n->dataFuture - minFuture) / (maxFuture - minFuture));
					score = scoreTemp + C * (double)sqrt(log(node->getNumVisits()) / n->getNumVisits());
				}
				else if (equalFlagFuture == true && equalFlag == false)
				{
					double scoreTemp = 1 - ((n->dataTemporary - minTemp) / (maxTemp - minTemp));
					score = scoreTemp + C * (double)sqrt(log(node->getNumVisits()) / n->getNumVisits());
				}
				else
				{
					double scoreTemp = 1 - ((n->dataTemporary - minTemp) / (maxTemp - minTemp));
					double scoreTemp1 = 1 - ((n->dataFuture - minFuture) / (maxFuture - minFuture));
					score = (1 - weight) * scoreTemp1 + weight * scoreTemp + C * (double)sqrt(log(node->getNumVisits()) / n->getNumVisits());
				}

				if (score >= bestScore)
				{
					bestScore = score;
					best = n;
				}
			}
		}
	}
	else
	{
		best = children[0];
	}

	return best;
}

Node* MCTS::expandNext(Node* node, int action)
{
	Node* newNode = new Node(++currentNodeID, node, action);
	node->childNo.push_back(action);
	std::sort(node->childNo.begin(), node->childNo.end());
	newNode->remainingVehicleNum.assign(node->remainingVehicleNum.begin(), node->remainingVehicleNum.end());
	newNode->remainingVehicleNum[action]--;
	newNode->childNum = 0;
	newNode->order.assign(node->order.begin(), node->order.end());
	newNode->order.push_back(vehicleNoLane[action][root->remainingVehicleNum[action] - node->remainingVehicleNum[action]]);
	int i = 0;
	for (std::vector<int>::iterator it = newNode->remainingVehicleNum.begin(); it != newNode->remainingVehicleNum.end(); ++it)
	{
		if (*it > 0)
		{
			newNode->remainingAction.push_back(i);
			newNode->childNum++;
		}
		i++;
	}
	for (int i = 0; i < 2 * MCTS::paraLaneNum6 * 2 * MCTS::paraLaneNum6; i++)
	{
		newNode->tempTAssignMin[i].assign(node->tempTAssignMin[i].begin(), node->tempTAssignMin[i].end());
		newNode->maxTAssignForEachZone[i] = node->maxTAssignForEachZone[i];
		newNode->maxActionIndexForEachZone[i] = node->maxActionIndexForEachZone[i];
	}
	
	newNode->dataTemporary = MCTS::ArrivalTimeCalculByIterativeSolution(newNode->order, MCTS::SimuVehicleDict, MCTS::TimeNow);
	node->addChild(newNode);
	return newNode;
}

void MCTS::simulate(Node* node)
{
	int times = 1;
	double result = 0;

	if (node->order.size() == vehNum)
	{
		node->score = node->dataTemporary;
		node->terminalFlag = 1;
		if (node->score < bestValue)
		{
			bestValue = node->score;
			bestOrder.clear();
			bestOrder.assign(node->order.begin(), node->order.end());
		}
	}
	else
	{
		for (int i = 0; i < times; i++)
		{
			vector<int> tempOrder = node->order;
			int depth = int(tempOrder.size());
			vector<int> tempRemainingVehicleNum = node->remainingVehicleNum;

			if (rule == 0)
			{
				int action;
				while (tempOrder.size() < vehNum)
				{
					action = randomNumberGenerator::uniformDistribution1To12(randomNumberGenerator::uniformGenerator);
					for (int k = 1; k <= 4 * MCTS::paraLaneNum6; k++)
					{
						int tempAction = action - k;
						if (tempAction < 0)
							tempAction += 4 * MCTS::paraLaneNum6;
						tempAction = tempAction % (4 * MCTS::paraLaneNum6);

						if (tempRemainingVehicleNum[tempAction] > 0)
						{
							tempOrder.push_back(vehicleNoLane[tempAction][root->remainingVehicleNum[tempAction] - tempRemainingVehicleNum[tempAction]]);
							tempRemainingVehicleNum[tempAction]--;
							break;
						}
					}
				}
				result = MCTS::ArrivalTimeCalculByIterativeSolution(tempOrder, MCTS::SimuVehicleDict, MCTS::TimeNow);

				if (result < node->score)
					node->score = result;

				if (result < bestValue)
				{
					bestValue = result;
					bestOrder.clear();
					bestOrder.assign(tempOrder.begin(), tempOrder.end());
				}

				if (updataPolicyFlag == 1)
					node->updatePolicyAverage(node->score, FIFOvalue);
				else
					node->updatePolicyOptimal(node->score, FIFOvalue);
			}
			else if (rule == 1)
			{
				int action;
				while (tempOrder.size() < vehNum)
				{
					action = randomNumberGenerator::uniformDistribution1To12(randomNumberGenerator::uniformGenerator);
					for (int k = 1; k <= 4 * MCTS::paraLaneNum6; k++)
					{
						int tempAction = action - k;
						if (tempAction < 0)
							tempAction += 4 * MCTS::paraLaneNum6;
						tempAction = tempAction % (4 * MCTS::paraLaneNum6);

						if (tempRemainingVehicleNum[tempAction] > 0)
						{
							tempOrder.push_back(vehicleNoLane[tempAction][root->remainingVehicleNum[tempAction] - tempRemainingVehicleNum[tempAction]]);
							tempRemainingVehicleNum[tempAction]--;
							break;
						}
					}
				}
				
				result = MCTS::ArrivalTimeCalculByIterativeSolution(tempOrder, MCTS::SimuVehicleDict, MCTS::TimeNow);

				if (result < node->score)
					node->score = result;

				if (result < bestValue)
				{
					bestValue = result;
					bestOrder.clear();
					bestOrder.assign(tempOrder.begin(), tempOrder.end());
				}

				if (updataPolicyFlag == 1)
					node->updatePolicyAverage(node->score, FIFOvalue);
				else
					node->updatePolicyOptimal(node->score, FIFOvalue);
			}
			else
			{
				vector<double> simulationTAssignMin[10 * 10];
				double maxSimulationTAssignForEachZone[10 * 10];
				int maxSimulationActionIndexForEachZone[10 * 10];
				for (int t = 0; t < 2 * MCTS::paraLaneNum6 * 2 * MCTS::paraLaneNum6; t++)
				{
					simulationTAssignMin[t].assign(node->tempTAssignMin[t].begin(), node->tempTAssignMin[t].end());
					maxSimulationTAssignForEachZone[t] = node->maxTAssignForEachZone[t];
					maxSimulationActionIndexForEachZone[t] = node->maxActionIndexForEachZone[t];
				}

				vector<int> simulationOrder = node->order;
				vector<int> remainingAction = node->remainingAction;
				while (simulationOrder.size() < vehNum)
				{
					depth = int(simulationOrder.size());
					double firstVehicleTime[10*10]; 
					int minTimeLabel[10 * 10];     

					for (int kk = 0; kk < 2 * MCTS::paraLaneNum6 * 2 * MCTS::paraLaneNum6; kk++)
					{
						firstVehicleTime[kk] = 10000;
						minTimeLabel[kk] = -1;
					}
					for (int possibleAction = 0; possibleAction < remainingAction.size(); possibleAction++)
					{
						tempOrder = simulationOrder;
						int tempLabel = vehicleNoLane[remainingAction[possibleAction]][root->remainingVehicleNum[remainingAction[possibleAction]] - tempRemainingVehicleNum[remainingAction[possibleAction]]];
						tempOrder.push_back(tempLabel);
					}

					bool failFlag = false;
					for (int possibleAction = 0; possibleAction < remainingAction.size(); possibleAction++)
					{
						int tempLabel = vehicleNoLane[remainingAction[possibleAction]][root->remainingVehicleNum[remainingAction[possibleAction]] - tempRemainingVehicleNum[remainingAction[possibleAction]]];
						bool labelFlag = 0;
						for (int allLable = 0; allLable < MCTS::SimuVehicleDict[tempLabel].TrajectoryCoverageConflictSubzonesArraySize; allLable++)
						{
							if (minTimeLabel[MCTS::SimuVehicleDict[tempLabel].TrajectoryCoverageConflictSubzonesArray[allLable]] != tempLabel)
							{
								labelFlag = 1;
								break;
							}
						}
						if (labelFlag == 0)
						{
							failFlag = true;
							simulationOrder.push_back(tempLabel);
							tempRemainingVehicleNum[remainingAction[possibleAction]]--;
							
							break;
						}
					}
					if (!failFlag)
					{
						int action = randomNumberGenerator::uniformDistribution1To12(randomNumberGenerator::uniformGenerator); 
						for (int k = 1; k <= 4*MCTS::paraLaneNum6; k++)
						{
							int tempAction = action - k;
							if (tempAction < 0)
								tempAction += 4 * MCTS::paraLaneNum6;
							tempAction = tempAction % (4 * MCTS::paraLaneNum6);

							if (tempRemainingVehicleNum[tempAction] > 0)
							{
								simulationOrder.push_back(vehicleNoLane[tempAction][root->remainingVehicleNum[tempAction] - tempRemainingVehicleNum[tempAction]]);
								tempRemainingVehicleNum[tempAction]--;
								
								break;
							}
						}
					}
					int possibleAction1 = 0;
					remainingAction.clear();
					for (std::vector<int>::iterator it = tempRemainingVehicleNum.begin(); it != tempRemainingVehicleNum.end(); ++it)
					{
						if (*it > 0)
						{
							remainingAction.push_back(possibleAction1);
						}
						possibleAction1++;
					}
				}

				depth = int(node->order.size());
				
				result = MCTS::ArrivalTimeCalculByIterativeSolution(tempOrder, MCTS::SimuVehicleDict, MCTS::TimeNow);

				if (result < node->score)
					node->score = result;

				if (updataPolicyFlag == 1)
					node->updatePolicyAverage(node->score, FIFOvalue);
				else
					node->updatePolicyOptimal(node->score, FIFOvalue);

				if (result < bestValue)
				{
					bestValue = result;
					bestOrder.clear();
					bestOrder.assign(simulationOrder.begin(), simulationOrder.end());
				}
				
			}
		}
	}
}


void MCTS::backProp(Node* node, double score)
{
	if (rule == 2)
	{
		while (node->getParent() != 0)
		{
			node = node->getParent();
			node->numVisits++;
			std::vector<Node*>& children = node->getChildren();

			node->terminalFlag = 1;
			for (Node* n : children)
			{
				if (!n->terminalFlag)
				{
					node->terminalFlag = 0;
					break;
				}
			}
		}
	}
	else
	{
		while (node->getParent() != 0)
		{
			node = node->getParent();
			if (updataPolicyFlag == 1)
				node->updatePolicyAverage(node->score, FIFOvalue);
			else
				node->updatePolicyOptimal(node->score, FIFOvalue);

			std::vector<Node*>& children = node->getChildren();

			node->terminalFlag = 1;
			for (Node* n : children)
			{
				if (!n->terminalFlag)
				{
					node->terminalFlag = 0;
					break;
				}
			}
		}
	}
}

double MCTS::ArrivalTimeCalculByIterativeSolution(list<int> aPassingOrderList, map<int, Vehicle> aSimuVehicleDict, double aTimeNow)
{
	ArrivalTime MyArrivalTime = *new ArrivalTime(
		MCTS::ConflictSubzonesLastPassedVehicleArrivalTime,
		MCTS::paraLaneNum6
	);

	double JTotalDelay = MyArrivalTime.PassingOrderToTrajectoryInterPretaton(
		aPassingOrderList,
		aSimuVehicleDict,
		aTimeNow
	);

	return JTotalDelay;
}

double MCTS::ArrivalTimeCalculByIterativeSolution(vector<int> aPassingOrderList, map<int, Vehicle> aSimuVehicleDict, double aTimeNow)
{
	ArrivalTime MyArrivalTime = *new ArrivalTime(
		MCTS::ConflictSubzonesLastPassedVehicleArrivalTime,
		MCTS::paraLaneNum6
	);

	list<int> PassingOrderList;
	for (vector<int>::iterator IntIter = aPassingOrderList.begin(); IntIter != aPassingOrderList.end(); IntIter++)
	{
		PassingOrderList.push_back(*IntIter);
	}
	double JTotalDelay = MyArrivalTime.PassingOrderToTrajectoryInterPretaton(
		PassingOrderList,
		aSimuVehicleDict,
		aTimeNow
	);

	return JTotalDelay;
}

double MCTS::ObjectiveValueUpdateBasedOnNodeInformation(
	vector<Vehicle>& vehicleSet, 
	vector<int> order, 
	int depth, 
	vector <double>* tempTAssignMin1, 
	double* maxTAssignForEachZone1, 
	int* maxActionIndexForEachZone1)
{
	return 0;
}