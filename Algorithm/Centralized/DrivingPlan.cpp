#include "DrivingPlan.h"

DrivingPlan::DrivingPlan()
{
	DrivingPlan::_count = 0;
}

DrivingPlan::~DrivingPlan()
{
}

void DrivingPlan::SetInitInformation(int aStrategyId, int aLeadingVehicleId, int aMappingVehicleId)
{
	switch (aStrategyId)
	{
	case 1:
		DrivingPlan::StrategyType = "Free Driving";
		break;
	case 2:
		DrivingPlan::StrategyType = "Leading Vehicle Following";
		break;
	case 3:
		DrivingPlan::StrategyType = "Mapping Vehicle Following";
		break;
	default:
		DrivingPlan::StrategyType = "Mix Following";
		break;
	}

	DrivingPlan::LeadingVehicleId = aLeadingVehicleId;
	DrivingPlan::MappingVehicleId = aMappingVehicleId;
}

void DrivingPlan::Add(BPointCoordinate aPosition, double aPoseAngle, double aSpeed, double aAngularSpeed)
{
	DrivingPlan::_count++;

	DrivingPlan::AngularSpeedList.push_back(aAngularSpeed);
	DrivingPlan::SpeedList.push_back(aSpeed);
	DrivingPlan::PoseAngleList.push_back(aPoseAngle);
	DrivingPlan::PositionList.push_back(*new BPointCoordinate(aPosition.X, aPosition.Y));
}

void DrivingPlan::Run(double& aSpeed, double& aAngularSpeed)
{
	aSpeed = DrivingPlan::SpeedList.front();
	aAngularSpeed = DrivingPlan::AngularSpeedList.front();

	DrivingPlan::SpeedList.pop_front();
	DrivingPlan::AngularSpeedList.pop_front();

	DrivingPlan::PoseAngleList.pop_front();
	DrivingPlan::PositionList.pop_front();

	DrivingPlan::DistanceList.pop_front();
	
	DrivingPlan::_count--;
}

int DrivingPlan::GetCount()
{
	return DrivingPlan::_count;
	return DrivingPlan::_count;
}

DrivingPlan DrivingPlan::DeepCopy()
{
	DrivingPlan localCopyPlan = *new DrivingPlan();
	localCopyPlan.AngularSpeedList = *new list<double> (DrivingPlan::AngularSpeedList);
	localCopyPlan.SpeedList = *new list<double>(DrivingPlan::SpeedList);
	localCopyPlan.PoseAngleList = *new list<double>(DrivingPlan::PoseAngleList);
	localCopyPlan.PositionList = *new list<BPointCoordinate>(DrivingPlan::PositionList);
	localCopyPlan.DistanceList = *new list<double>(DrivingPlan::DistanceList);

	localCopyPlan._count = DrivingPlan::_count;
	localCopyPlan.WaitingOrder = DrivingPlan::WaitingOrder;
	localCopyPlan.LeadingVehicleId = DrivingPlan::LeadingVehicleId;
	localCopyPlan.MappingVehicleId = DrivingPlan::MappingVehicleId;
	localCopyPlan.StrategyType = DrivingPlan::StrategyType;

	return localCopyPlan;

}

void DrivingPlan::Clear()
{
	DrivingPlan::AngularSpeedList.clear();
	DrivingPlan::SpeedList.clear();
	DrivingPlan::PoseAngleList.clear();
	DrivingPlan::PositionList.clear();
	DrivingPlan::_count = 0;
}