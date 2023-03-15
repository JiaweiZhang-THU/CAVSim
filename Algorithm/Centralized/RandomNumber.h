#pragma once
#include <iostream>
#include <random>
using namespace std;

namespace simulationSetting
{
	extern double simulationTimeStep;
	extern double arrivalRate;
}

namespace randomNumberGenerator
{
	extern default_random_engine poissonGenerator;
	extern poisson_distribution<int> poissonDistribution;
	
	extern random_device rd;
	extern mt19937 uniformGenerator;
	extern uniform_int_distribution<int> uniformDistribution1To2;
	extern uniform_int_distribution<int> uniformDistribution1To3;
	extern uniform_int_distribution<int> uniformDistribution1To4;
	extern uniform_int_distribution<int> uniformDistribution1To12;
	extern uniform_real_distribution<> uniformDistribution0To1;
}
