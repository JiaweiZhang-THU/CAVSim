#pragma once
#include<vector>
#include<limits>
#include<iostream>

using namespace std;
class Node
{
public:
	int id;
	Node* parent;
	vector<Node*> children;
	int action;

	double data;

	double dataTemporary;
	double dataFuture;
	int numVisits;
	double score;
	int selfLink;

	bool terminalFlag; 

	vector<int> childNo;

	vector<int> remainingVehicleNum; 

	vector<int> remainingAction;

	int childNum; 

	vector<int> order;

	vector<double> tempTAssignMin[10 * 10];

	double maxTAssignForEachZone[10 * 10];

	int maxActionIndexForEachZone[10 * 10];


	Node(unsigned int a_id, Node* a_parent, int a_action)
	{
		Node::id = a_id;
		Node::parent = a_parent;
		Node::action = a_action;
		Node::score = (std::numeric_limits<double>::max)();
	}


	unsigned int getID() {
		return id;
	}


	Node* getParent() {
		return parent;
	}


	std::vector<Node*>& getChildren() {
		return children;
	}

	void addChild(Node* child) {
		children.push_back(child);
	}


	bool shouldExpand() {
		bool result = children.empty() || children.size() < childNum;
		return result;
	}


	void updatePolicyAverage(double score, double FIFOValue)
	{
		double temp = 0;
		temp = (FIFOValue - score) / FIFOValue * 2;
		if (temp < 0)
			temp = 0;

		data = (data * numVisits + temp) / ((double)numVisits + 1);
		numVisits++;
	}

	void updatePolicyRule(double score, double FIFOValue)
	{
		double temp = 0;
		temp = (FIFOValue - score) / FIFOValue;
		if (temp < 0)
			temp = 0;

		data = temp;
		

		numVisits++;
	}

	void updatePolicyOptimal(double score, double FIFOValue)
	{
		double temp = 0;
		temp = (FIFOValue - score) / FIFOValue * 2;
		if (temp < 0)
			temp = 0;

		if (temp > data)
			data = temp;
		numVisits++;
	}


	float getAvgScore() {
		return float(score / numVisits);
	}


	int getNumVisits() {
		return numVisits;
	}


	~Node()
	{
		for (Node* child : children)
			delete child;
	}
};
