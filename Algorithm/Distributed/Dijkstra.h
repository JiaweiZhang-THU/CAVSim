#pragma once
#ifndef _DIJKSTRA_
#define _DIJKSTRA_

#include<iostream>

using namespace std;

#define MaxSize 100

class Dijkstra
{
public:
	Dijkstra();
	~Dijkstra();

	Dijkstra(int a_vexnum, int a_arcnum);


	int vexnum; 

	int arcnum; 

	int PathNodeNum;
	int PathNodeList[MaxSize];
	double PathDistance;


	double AdjacencyMatrixDistance[MaxSize][MaxSize];

	void printPath(int parent[], int j);

	int minimumDist(double dist[], bool Tset[], int count);

	double findShortestPath(int src, int end, int flag = 1);

private:

};


#endif // !_DIJKSTRA_