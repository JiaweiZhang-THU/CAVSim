#include "Dijkstra.h"


Dijkstra::Dijkstra()
{
}

Dijkstra::Dijkstra(int a_vexnum, int a_arcnum)
{
    Dijkstra::vexnum = a_vexnum;
    Dijkstra::arcnum = a_arcnum;
}

Dijkstra::~Dijkstra()
{
}

void Dijkstra::printPath(int parent[], int j)
{
    if (parent[j] == -1)
        return;
    printPath(parent, parent[j]);
    Dijkstra::PathNodeNum += 1;
    Dijkstra::PathNodeList[Dijkstra::PathNodeNum] = j - 1;
}

int Dijkstra::minimumDist(double dist[], bool Tset[], int count)
{
    int i, min = INT_MAX, index;
    for (i = 1; i <= count; i++) {
        if (Tset[i] == false && dist[i] <= min) {
            min = int(dist[i]);
            index = i;
        }
    }
    return index;
}

double Dijkstra::findShortestPath(int src, int end, int flag)
{
    src += 1;
    end += 1;
   
    int i, j, m;
    double dist[MaxSize];
    bool Tset[MaxSize];
    int parent[MaxSize];
    for (i = 1; i <= Dijkstra::vexnum; i++) {
        dist[i] = INT_MAX;
        Tset[i] = false;
        parent[i] = -1;
    }
    dist[src] = 0;

    for (i = 1; i <= Dijkstra::vexnum; i++) {
        m = minimumDist(dist, Tset, Dijkstra::vexnum);
        Tset[m] = true;
        for (j = 1; j <= Dijkstra::vexnum; j++) {
            if (!Tset[j] && Dijkstra::AdjacencyMatrixDistance[m][j] && dist[m] != INT_MAX
                && dist[m] + Dijkstra::AdjacencyMatrixDistance[m][j] < dist[j]) {
                parent[j] = m;
                dist[j] = dist[m] + Dijkstra::AdjacencyMatrixDistance[m][j];
            }
        }
    }
    Dijkstra::PathNodeNum = 0;
    if (flag) {
        Dijkstra::PathNodeList[Dijkstra::PathNodeNum] = src - 1;
        printPath(parent, end);
    }
    Dijkstra::PathDistance = dist[end];
    Dijkstra::PathNodeNum += 1;

    return dist[end];
}