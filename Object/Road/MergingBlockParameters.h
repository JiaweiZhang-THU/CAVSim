#pragma once
#include <math.h>
#include "BPointCoordinate.h"

int paraNumOfLanesOnMainRoad = 2;

double paraLaneWidthMerging = 4;

double paraAngleOfMergingLane = acos(-1) * 30 / 180;

double paraEnterMainRoadLength = 200;
double paraExitMainRoadLength = 200;
double paraConflictRoadLength = paraLaneWidthMerging / sin(paraAngleOfMergingLane) + 50;
double paraMergingLaneLength = 120;


BPointCoordinate paraEnterMainRoadCenter(paraEnterMainRoadLength / 2, 0); 