#include"FollowingBehaviors.h"
#include"FollowingBehaviorsParameters.h"

FollowingBehaviors::FollowingBehaviors()
{

    FollowingBehaviors::DelayInTime = paraDelayInTime;
    FollowingBehaviors::DeltaTime = paraDeltaTime2;
    FollowingBehaviors::MaintainingDistance = paraMaintainingDistance;

	FollowingBehaviors::VeryLowSpeed = 0.001;
	FollowingBehaviors::IsTimeToConsiderMapping = 150;

	FollowingBehaviors::Sigma = 1;
}

FollowingBehaviors::~FollowingBehaviors()
{
}

void FollowingBehaviors::AgentBasedFollowingBehavior(double noise, double headDist, double headV, double& currSpeed, Vehicle simuVehicle)
{

    headDist += headV * FollowingBehaviors::DelayInTime + noise;


    double consideringDistance = fmin(headDist,
        headDist + (headV * headV - currSpeed * currSpeed) / (2 * simuVehicle.MaxStraightAccel));

    currSpeed = consideringDistance < FollowingBehaviors::MaintainingDistance
        ? fmax(VeryLowSpeed, currSpeed - FollowingBehaviors::DeltaTime * simuVehicle.MaxStraightAccel)
        : fmin(simuVehicle.MaxStraightSpeed, currSpeed + FollowingBehaviors::DeltaTime * simuVehicle.MaxStraightAccel);
}


void FollowingBehaviors::PlanBasedFollowingBehaviorWhenMapping(double noise, double headDist, double headV, double& currSpeed, Vehicle simuVehicle, double selfDist)
{
    if (selfDist < IsTimeToConsiderMapping)
    {

        headDist += headV * FollowingBehaviors::DelayInTime + noise;

        double consideringDistance = fmin(headDist,
            headDist + (headV * headV - currSpeed * currSpeed) / (2 * simuVehicle.MaxStraightAccel));

        currSpeed = consideringDistance < 0
            ? fmax(VeryLowSpeed, currSpeed - FollowingBehaviors::DeltaTime * simuVehicle.MaxStraightAccel)
            : fmin(simuVehicle.MaxStraightSpeed, currSpeed + FollowingBehaviors::DeltaTime * simuVehicle.MaxStraightAccel);
    }
    else
    {
        currSpeed = fmin(simuVehicle.MaxStraightSpeed, currSpeed + FollowingBehaviors::DeltaTime * simuVehicle.MaxStraightAccel);
    }
}