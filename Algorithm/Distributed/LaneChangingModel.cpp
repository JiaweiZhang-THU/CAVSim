#include "LaneChangingModel.h"
#include "LaneChangingModelParameters.h"

LaneChangingModel::LaneChangingModel()
{
	LaneChangingModel::LaneNum = paraLaneNumAtLaneChangingModel;
}

LaneChangingModel::~LaneChangingModel()
{
}


int LaneChangingModel::KeepInCurrentLane(list<Vehicle> aPerceptionSurroundingVehiclesList, Vehicle aAgentVehicle)
{
	int localLaneChangingAction = 0;
	return localLaneChangingAction;
}



int LaneChangingModel::RandomLaneChangingModel(list<Vehicle> aPerceptionSurroundingVehiclesList, Vehicle aAgentVehicle)
{
	double random_value = LaneChangingModel::MyRandom.randomUniform(0, 1);
	if (random_value < 0.0025)
	{
		return 1;
	}
	else
	{
		if (random_value > 0.9975)
		{
			return -1;
		}
		else
		{
			return 0;
		}
	}
}