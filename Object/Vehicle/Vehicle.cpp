#include "Vehicle.h"
#include "VehicleParameters.h"
#include <random>
#include <ctime>

Vehicle::Vehicle(void)
{
	Vehicle::Length = paraVehicleLength;
	Vehicle::Width = paraVehicleWidth;
	Vehicle::Type = paraVehicleType;

	Vehicle::MaxStraightSpeed = paraMaxStraightSpeed;
	Vehicle::MinStraightSpeed = paraMinStraightSpeed;

	Vehicle::MaxStraightAccel = paraMaxStraightAccel;
	Vehicle::MinStraightAccel = paraMinStraightAccel;

	Vehicle::MaxSteerSpeed = paraMaxSteerSpeed;
	Vehicle::MaxAngularAccel = paraMaxStraightAccel;
	Vehicle::MaxAngular = paraMaxAngular;

	Vehicle::Location = paraDefaultLocation;
	Vehicle::Speed = paraDefaultSpeed;
	Vehicle::SpeedAtLastStep = 0;
	Vehicle::SpeedBeforeNSecons = paraDefaultSpeed;
	Vehicle::AngularSpeed = paraDefaultAngularSpeed;
	Vehicle::PoseAngle = paraDefaultPoseAngle;
	Vehicle::Direction = paraDefaultRoadDirection;

	Vehicle::NewList();
	Vehicle::RecodeSpeedList = list<double>();

	Vehicle::LastLaneChangeAction = 0;
	Vehicle::DistannceFromTargetLane = 0;

	Vehicle::RoadBlockType = 0;

	Vehicle::IntersectionPointer = NULL;
	Vehicle::LaneUnitPointer = NULL;
	Vehicle::StraightLanePointer = NULL;

	BRectangle localVehicleRectangle(Vehicle::Width, Vehicle::Length, Vehicle::Location, Vehicle::Direction + Vehicle::PoseAngle);
	Vehicle::VehicleRectangle = localVehicleRectangle;
	Vehicle::IsCollide = false;

	Vehicle::TrajectoryCoverageConflictSubzonesArraySize = 0;
	for (int i = 0; i < 40; i++)
	{
		Vehicle::TrajectoryCoverageConflictSubzonesArray[i] = 0;
	}

	Vehicle::SteeringType = 0;
	Vehicle::SteeringRadius = 0;

	Vehicle::EnteringBlockId = -1;
	Vehicle::TargetBlockId = -1;

	Vehicle::TimeStep = 0;

	Vehicle::PlanState = VehiclePlanState::Free;

	Vehicle::InIntersectionRadius = VehicleInIntersectionRadius::Outter;

	Vehicle::StrategyMode = "";
	Vehicle::HeadId = 0;
	Vehicle::ConflictId = 0;
	
	Vehicle::PassingTime = 0;

	Vehicle::MyDrivingPlanPointer = NULL;

	Vehicle::HeadwayDistance = -1;

	Vehicle::SpeedSetPastNSecondsList.push_front(Vehicle::Speed);

	Vehicle::CanMoveForward = true;

	Vehicle::ResetIDMParameters();

	Vehicle::Acceleration = 0;
	Vehicle::AccelerationAtLastStep = 0;
	Vehicle::HeadwayBetweenTheEgoAndFronVehicle = 0;

	Vehicle::WhetherToSendRealtimeSpeed = false;

	Vehicle::TimeOfEnteringIntersectionCircleControlZone = -1;
	Vehicle::TimeOfEnteringIntersectionConflictZone = -1;
	Vehicle::TravelDistance = 0;
}

Vehicle::Vehicle(BPointCoordinate aInitialLocation, double aInitialPoseAngle)
{

	Vehicle::Length = paraVehicleLength;
	Vehicle::Width = paraVehicleWidth;
	Vehicle::Type = paraVehicleType;

	Vehicle::MaxStraightSpeed = paraMaxStraightSpeed;
	Vehicle::MinStraightSpeed = paraMinStraightSpeed;

	Vehicle::MaxStraightAccel = paraMaxStraightAccel;
	Vehicle::MinStraightAccel = paraMinStraightAccel;

	Vehicle::MaxSteerSpeed = paraMaxSteerSpeed;
	Vehicle::MaxAngularAccel = paraMaxStraightAccel;

	Vehicle::MaxAngular = paraMaxAngular;


	Vehicle::Location = aInitialLocation;
	Vehicle::Speed = paraDefaultSpeed;
	Vehicle::SpeedAtLastStep = 0;
	Vehicle::SpeedBeforeNSecons = paraDefaultSpeed;
	Vehicle::AngularSpeed = paraDefaultAngularSpeed;
	Vehicle::PoseAngle = aInitialPoseAngle;
	Vehicle::Direction = 0;


	Vehicle::NewList();
	Vehicle::RecodeSpeedList = list<double>();

	Vehicle::LastLaneChangeAction = 0;
	Vehicle::DistannceFromTargetLane = 0;

	Vehicle::RoadBlockType = 0;

	Vehicle::IntersectionPointer = NULL;
	Vehicle::LaneUnitPointer = NULL;
	Vehicle::StraightLanePointer = NULL;

	BRectangle localVehicleRectangle(Vehicle::Width, Vehicle::Length, Vehicle::Location, Vehicle::Direction + Vehicle::PoseAngle);
	Vehicle::VehicleRectangle = localVehicleRectangle;
	Vehicle::IsCollide = false;

	Vehicle::TrajectoryCoverageConflictSubzonesArraySize = 0;
	for (int i = 0; i < 40; i++)
	{
		Vehicle::TrajectoryCoverageConflictSubzonesArray[i] = 0;
	}

	Vehicle::SteeringType = 0;
	Vehicle::SteeringRadius = 0;

	Vehicle::TimeStep = 0; 

	Vehicle::PlanState = VehiclePlanState::Free;

	Vehicle::InIntersectionRadius = VehicleInIntersectionRadius::Outter;

	Vehicle::StrategyMode = "";
	Vehicle::HeadId = 0;
	Vehicle::ConflictId = 0;

	Vehicle::PassingTime = 0;

	Vehicle::MyDrivingPlanPointer = NULL;

	Vehicle::SpeedSetPastNSecondsList.push_front(Vehicle::Speed);

	Vehicle::CanMoveForward = true;

	Vehicle::ResetIDMParameters();

	Vehicle::Acceleration = 0;
	Vehicle::AccelerationAtLastStep = 0;
	Vehicle::HeadwayBetweenTheEgoAndFronVehicle = 0;

	Vehicle::WhetherToSendRealtimeSpeed = false;

	Vehicle::TimeOfEnteringIntersectionCircleControlZone = -1;

	Vehicle::TimeOfEnteringIntersectionConflictZone = -1;

	Vehicle::TravelDistance = 0;
}


Vehicle::Vehicle(BPointCoordinate aInitialLocation, double aInitialPoseAngle, double aDirection)
{
	
	Vehicle::Length = paraVehicleLength;
	Vehicle::Width = paraVehicleWidth;
	Vehicle::Type = paraVehicleType;

	Vehicle::MaxStraightSpeed = paraMaxStraightSpeed;
	Vehicle::MinStraightSpeed = paraMinStraightSpeed;

	Vehicle::MaxStraightAccel = paraMaxStraightAccel;
	Vehicle::MinStraightAccel = paraMinStraightAccel;

	Vehicle::MaxSteerSpeed = paraMaxSteerSpeed;
	Vehicle::MaxAngularAccel = paraMaxStraightAccel;

	Vehicle::MaxAngular = paraMaxAngular;


	Vehicle::Location = aInitialLocation;
	Vehicle::Speed = paraDefaultSpeed;
	Vehicle::SpeedAtLastStep = 0;
	Vehicle::SpeedBeforeNSecons = paraDefaultSpeed;
	Vehicle::AngularSpeed = paraDefaultAngularSpeed;
	Vehicle::PoseAngle = aInitialPoseAngle;
	Vehicle::Direction = aDirection;
	

	Vehicle::NewList();
	Vehicle::RecodeSpeedList = list<double>();

	Vehicle::LastLaneChangeAction = 0;
	Vehicle::DistannceFromTargetLane = 0;

	Vehicle::RoadBlockType = 0;

	Vehicle::IntersectionPointer = NULL;
	Vehicle::LaneUnitPointer = NULL;
	Vehicle::StraightLanePointer = NULL;

	BRectangle localVehicleRectangle(Vehicle::Width, Vehicle::Length, Vehicle::Location, Vehicle::Direction + Vehicle::PoseAngle);
	Vehicle::VehicleRectangle = localVehicleRectangle;
	Vehicle::IsCollide = false;

	Vehicle::TrajectoryCoverageConflictSubzonesArraySize = 0;
	for (int i = 0; i < 40; i++)
	{
		Vehicle::TrajectoryCoverageConflictSubzonesArray[i] = 0;
	}

	Vehicle::SteeringType = 0;
	Vehicle::SteeringRadius = 0;

	Vehicle::TimeStep = 0;

	Vehicle::PlanState = VehiclePlanState::Free;

	Vehicle::InIntersectionRadius = VehicleInIntersectionRadius::Outter;

	Vehicle::StrategyMode = "";
	Vehicle::HeadId = 0;
	Vehicle::ConflictId = 0;

	Vehicle::PassingTime = 0;

	Vehicle::MyDrivingPlanPointer = NULL;

	Vehicle::SpeedSetPastNSecondsList.push_front(Vehicle::Speed);

	Vehicle::CanMoveForward = true;

	Vehicle::ResetIDMParameters();

	Vehicle::Acceleration = 0;
	Vehicle::AccelerationAtLastStep = 0;
	Vehicle::HeadwayBetweenTheEgoAndFronVehicle = 0;

	Vehicle::WhetherToSendRealtimeSpeed = false;

	Vehicle::TimeOfEnteringIntersectionCircleControlZone = -1;

	Vehicle::TimeOfEnteringIntersectionConflictZone = -1;

	Vehicle::TravelDistance = 0;
}

void Vehicle::NewList(void)
{
	Vehicle::SpeedList = list<double>();
	Vehicle::AngularSpeedList = list<double>();
	Vehicle::XLocationList = list<double>();
	Vehicle::YLocationList = list<double>();
}

void Vehicle::ClearList(void)
{
	Vehicle::SpeedList.clear();
	Vehicle::AngularSpeedList.clear();
	Vehicle::XLocationList.clear();
	Vehicle::YLocationList.clear();
}

void Vehicle::AddList(double _Speed, double _AngularSpeed, double _XLocation, double _YLocation)
{
	Vehicle::SpeedList.push_back(_Speed);
	Vehicle::AngularSpeedList.push_back(_AngularSpeed);
	Vehicle::XLocationList.push_back(_XLocation);
	Vehicle::YLocationList.push_back(_YLocation);
}

string Vehicle::GetName()
{
	string Name = "\tVehicle: " + std::to_string(Vehicle::Id) + "\tLocation: (" + std::to_string(Vehicle::Location.X) + "," + std::to_string(Vehicle::Location.Y) + ")\tPoseAngle: " + std::to_string(Vehicle::PoseAngle) + "\n";
	return Name;
}

void Vehicle::ResetIDMParameters() 
{
	std::default_random_engine gen((unsigned int)time(NULL));
	std::normal_distribution<double> Norm_T0(IDM_Parameters_T0_E, IDM_Parameters_T0_sigma);
	Vehicle::IDM_Parameters_T0 = Norm_T0(gen);
	if (Vehicle::IDM_Parameters_T0 < 0.5)
	{
		Vehicle::IDM_Parameters_T0 = 0.5;
	}
	std::normal_distribution<double> Norm_b(IDM_Parameters_b_E, IDM_Parameters_b_sigma);
	Vehicle::IDM_Parameters_b = Norm_b(gen);
	if (Vehicle::IDM_Parameters_b < 1)
	{
		Vehicle::IDM_Parameters_b = 1;
	}
}

void Vehicle::UpdateVehicleLocation(double DeltaTime)
{
	Vehicle::Location.X += Vehicle::Speed * DeltaTime * cos(Vehicle::Direction + Vehicle::PoseAngle);
	Vehicle::Location.Y += Vehicle::Speed * DeltaTime * sin(Vehicle::Direction + Vehicle::PoseAngle);

	Vehicle::TravelDistance += Vehicle::Speed * DeltaTime;

	Vehicle::UpdateDistannceFromTargetLane(DeltaTime);
	Vehicle::UpdateVehicleRectangularBox();

	Vehicle::UpdateLeftDistance(DeltaTime);
}

void Vehicle::UpdateVehicleLocationForStringStability(double DeltaTime)
{
	Vehicle::Location.X += (Vehicle::SpeedAtLastStep * DeltaTime + 0.5 * Vehicle::AccelerationAtLastStep * DeltaTime * DeltaTime) * cos(Vehicle::Direction + Vehicle::PoseAngle);
	Vehicle::Location.Y += (Vehicle::SpeedAtLastStep * DeltaTime + 0.5 * Vehicle::AccelerationAtLastStep * DeltaTime * DeltaTime) * sin(Vehicle::Direction + Vehicle::PoseAngle);

	Vehicle::TravelDistance += Vehicle::SpeedAtLastStep * DeltaTime + 0.5 * Vehicle::AccelerationAtLastStep * DeltaTime * DeltaTime;

	Vehicle::UpdateDistannceFromTargetLane(DeltaTime);
	Vehicle::UpdateVehicleRectangularBox();

	Vehicle::UpdateLeftDistance(DeltaTime);

	Vehicle::SpeedAtLastStep = Vehicle::Speed;
	Vehicle::AccelerationAtLastStep = Vehicle::Acceleration;
}

void Vehicle::UpdateVehicleRectangularBox(void)
{
	Vehicle::VehicleRectangle.UpdateVertex(Vehicle::Location,Vehicle::Direction + Vehicle::PoseAngle);
}

void Vehicle::UpdateDistannceFromTargetLane(double DeltaTime)
{
	Vehicle::DistannceFromTargetLane -= Vehicle::Speed * DeltaTime * abs(sin(Vehicle::PoseAngle));
	if (abs(Vehicle::DistannceFromTargetLane) < paraDistanceNoise)
	{
		Vehicle::DistannceFromTargetLane = 0;
	}
}

void Vehicle::SceneNormalization(list<Vehicle>& aVehiclesList, Vehicle& aVehicle)
{
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = aVehiclesList.begin(); localVehicleIterator != aVehiclesList.end(); localVehicleIterator++)
	{
		localVehicleIterator->Location.X -= aVehicle.Location.X;
		localVehicleIterator->Location.Y -= aVehicle.Location.Y;

		localVehicleIterator->Location.X = localVehicleIterator->Location.X * cos(-aVehicle.Direction) - localVehicleIterator->Location.Y * sin(-aVehicle.Direction);
		localVehicleIterator->Location.Y = localVehicleIterator->Location.X * sin(-aVehicle.Direction) + localVehicleIterator->Location.Y * cos(-aVehicle.Direction);
		localVehicleIterator->Direction -= aVehicle.Direction;
		while (localVehicleIterator->Direction < 0)
		{
			localVehicleIterator->Direction += 2 * acos(-1);
		}
		while (localVehicleIterator->Direction > (2 * acos(-1)))
		{
			localVehicleIterator->Direction -= 2 * acos(-1);
		}
	}

	aVehicle.Location.X -= aVehicle.Location.X;
	aVehicle.Location.Y -= aVehicle.Location.Y;
}


bool Vehicle::IsCollideWithOtherVehicles(list<Vehicle> aSurroundingVehicles)
{
	bool localIsCollide = false;
	if (aSurroundingVehicles.size() <= 0)
	{
		return localIsCollide;
	}
	for (list<Vehicle>::iterator localVehicleIterator = aSurroundingVehicles.begin(); localVehicleIterator != aSurroundingVehicles.end(); localVehicleIterator++)
	{
		if (localVehicleIterator->Id == Vehicle::Id)
		{
			continue;
		}
		if (Vehicle::VehicleRectangle.IsIntersect(localVehicleIterator->VehicleRectangle) == true)
		{
			localIsCollide = true;
			return localIsCollide;
		}
	}
	return localIsCollide;
}

void Vehicle::UpdateLeftDistance(double DeltaTime)
{
	Vehicle::LeftLaneDistance -= Vehicle::Speed * DeltaTime;
	Vehicle::LeftDistance = Vehicle::LeftLaneDistance + Vehicle::LeftInterDistance;

	Vehicle::NoiseInDistance = Normal();
}

double Vehicle::Normal()
{
	return 0;
}

void Vehicle::UpdateLassSpeedSetPastNSeconds(int aSimulatingStep)
{
	if (aSimulatingStep % 10 == 0)
	{
		Vehicle::SpeedSetPastNSecondsList.push_front(Vehicle::Speed);
		if (Vehicle::SpeedSetPastNSecondsList.size() > paraPerceptualDelayStepsNumInVehicle)
		{
			Vehicle::SpeedSetPastNSecondsList.pop_back();
			Vehicle::SpeedBeforeNSecons = Vehicle::SpeedSetPastNSecondsList.back();
		}
		else
		{
			Vehicle::SpeedBeforeNSecons = Vehicle::SpeedSetPastNSecondsList.back();
		}
	}
}

void Vehicle::UpdatePerceptionSurroundingIntersectionVehicles(list<Vehicle> aPerceptionSurroundingVehiclesList, Vehicle& aVehicle)
{
	aVehicle.HeadId = 0;
	aVehicle.HeadDistance = DBL_MAX;
	aVehicle.HeadSpeed = DBL_MAX;
	if (aVehicle.LeftLaneDistance <= 0)
	{
		return;
	}
	Vehicle aTempVehicle = aVehicle;
	Vehicle::SceneNormalization(aPerceptionSurroundingVehiclesList, aTempVehicle);
	list<Vehicle>::iterator localVehicleIterator;
	double MinLeadingDistance = DBL_MAX;
	if (aPerceptionSurroundingVehiclesList.size() > 0)
	{
		for (localVehicleIterator = aPerceptionSurroundingVehiclesList.begin(); localVehicleIterator != aPerceptionSurroundingVehiclesList.end(); localVehicleIterator++)
		{
			if (localVehicleIterator->Location.X > 1e-4 && localVehicleIterator->Location.X < aTempVehicle.LeftLaneDistance && localVehicleIterator->BlockId == aVehicle.BlockId && localVehicleIterator->LaneId == aVehicle.LaneId and localVehicleIterator->Id != aVehicle.Id)
			{
				if (localVehicleIterator->Location.X - paraVehicleLength < MinLeadingDistance)
				{
					MinLeadingDistance = localVehicleIterator->Location.X - paraVehicleLength;
					aVehicle.HeadId = localVehicleIterator->Id;
					aVehicle.HeadDistance = MinLeadingDistance;
					aVehicle.HeadSpeed = localVehicleIterator->Speed;
				}
			}
		}
	}
}



list<Vehicle> Vehicle::MiddleFollowingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList)
{
	list<Vehicle> SurroundingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = aNormlinzePerceptionSurroundingVehiclesList.begin(); localVehicleIterator != aNormlinzePerceptionSurroundingVehiclesList.end(); localVehicleIterator++)
	{
		if (localVehicleIterator->LaneId == Vehicle::LaneId and localVehicleIterator->Location.X < 0)
		{
			SurroundingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return SurroundingVehicleList;
}

list<Vehicle> Vehicle::MiddleLeadingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList)
{
	list<Vehicle> LeadingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = aNormlinzePerceptionSurroundingVehiclesList.begin(); localVehicleIterator != aNormlinzePerceptionSurroundingVehiclesList.end(); localVehicleIterator++)
	{
		if (localVehicleIterator->LaneId == Vehicle::LaneId and localVehicleIterator->Location.X > 0)
		{
			LeadingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return LeadingVehicleList;
}


list<Vehicle> Vehicle::LeftFollowingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList)
{
	list<Vehicle> SurroundingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = aNormlinzePerceptionSurroundingVehiclesList.begin(); localVehicleIterator != aNormlinzePerceptionSurroundingVehiclesList.end(); localVehicleIterator++)
	{
		if (localVehicleIterator->LaneId == Vehicle::LaneId + 1 and localVehicleIterator->Location.X < 0)
		{
			SurroundingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return SurroundingVehicleList;
}

list<Vehicle> Vehicle::LeftLeadingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList)
{
	list<Vehicle> LeadingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = aNormlinzePerceptionSurroundingVehiclesList.begin(); localVehicleIterator != aNormlinzePerceptionSurroundingVehiclesList.end(); localVehicleIterator++)
	{
		if (localVehicleIterator->LaneId == Vehicle::LaneId + 1 and localVehicleIterator->Location.X > 0)
		{
			LeadingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return LeadingVehicleList;
}



list<Vehicle> Vehicle::RightFollowingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList)
{
	list<Vehicle> SurroundingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = aNormlinzePerceptionSurroundingVehiclesList.begin(); localVehicleIterator != aNormlinzePerceptionSurroundingVehiclesList.end(); localVehicleIterator++)
	{
		if (localVehicleIterator->LaneId == Vehicle::LaneId - 1 and localVehicleIterator->Location.X < 0)
		{
			SurroundingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return SurroundingVehicleList;
}

list<Vehicle> Vehicle::RightLeadingVehicle(list<Vehicle> aNormlinzePerceptionSurroundingVehiclesList)
{
	list<Vehicle> LeadingVehicleList;
	list<Vehicle>::iterator localVehicleIterator;
	for (localVehicleIterator = aNormlinzePerceptionSurroundingVehiclesList.begin(); localVehicleIterator != aNormlinzePerceptionSurroundingVehiclesList.end(); localVehicleIterator++)
	{
		if (localVehicleIterator->LaneId == Vehicle::LaneId - 1 and localVehicleIterator->Location.X > 0)
		{
			LeadingVehicleList.push_back(*localVehicleIterator);
		}
	}
	return LeadingVehicleList;
}


Vehicle Vehicle::FindFollowingVehicle(list<Vehicle> aFollowingVehicleList)
{
	Vehicle localFollowingVehicle = aFollowingVehicleList.front();
	list<Vehicle>::iterator localVehicleIterator;

	for (localVehicleIterator = aFollowingVehicleList.begin(); localVehicleIterator != aFollowingVehicleList.end(); localVehicleIterator++)
	{
		if (localFollowingVehicle.Location.X < localVehicleIterator->Location.X)
		{
			localFollowingVehicle = *localVehicleIterator;
		}
	}
	return localFollowingVehicle;
}

Vehicle Vehicle::FindLeadingVehicle(list<Vehicle> aFollowingVehicleList)
{
	Vehicle localLeadingVehicle = aFollowingVehicleList.front();
	list<Vehicle>::iterator localVehicleIterator;

	for (localVehicleIterator = aFollowingVehicleList.begin(); localVehicleIterator != aFollowingVehicleList.end(); localVehicleIterator++)
	{
		if (localLeadingVehicle.Location.X > localVehicleIterator->Location.X)
		{
			localLeadingVehicle = *localVehicleIterator;
		}
	}
	return localLeadingVehicle;
}

void Vehicle::UpdateSpeedSentToFollowingVehicles()
{
	Vehicle::SpeedSentToFollowingVehicles = Vehicle::Speed;
	Vehicle::WhetherToSendRealtimeSpeed = true;
}
