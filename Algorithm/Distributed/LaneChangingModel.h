#ifndef _LANE_CHANGING_MODEL_
#define _LANE_CHANGING_MODEL_
#pragma once
#include<iostream>
#include "../../Object/Vehicle/Vehicle.h"
#include "CarFollowing_OVM.h"
#include "CarFollowing_Secondorder.h"

#include "../Others/cls_random.h"

class LaneChangingModel
{
public:
	LaneChangingModel();

	~LaneChangingModel();

	cls_random MyRandom;

	int LaneNum;

	int KeepInCurrentLane(list<Vehicle> aPerceptionSurroundingVehiclesList, Vehicle aAgentVehicle);

	int RandomLaneChangingModel(list<Vehicle> aPerceptionSurroundingVehiclesList, Vehicle aAgentVehicle);

	int ChosenCarFollowingModel;

	CarFollowing_Secondorder SecondorderModel;

	CarFollowing_OVM OVMModel;

};
#endif // !_LANE_CHANGING_MODEL_


