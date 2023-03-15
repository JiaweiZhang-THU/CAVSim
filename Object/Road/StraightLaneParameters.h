#pragma once
#ifndef _STRAIGHTLANEBLOCKPARAMETERS_
#define _STRAIGHTLANEBLOCKPARAMETERS_

#include "BPointCoordinate.h"

int paraDefaultLaneNumBasedOnBlocakParameter = 2;

int paraStraightLaneBlockNum = 30;

double paraStraightPerBlockLength = 200;

double paraPerLaneWidth = 4; 

double paraStraightLaneDirection = 0*acos(-1);

BPointCoordinate paraFirstBlockCenter(paraStraightPerBlockLength/2,0); 

#endif // !_STRAIGHTLANEBLOCKPARAMETERS_
