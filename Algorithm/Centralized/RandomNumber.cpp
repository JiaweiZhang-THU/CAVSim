#include <iostream>
#include <random>
using namespace std;

namespace simulationSetting
{
	double simulationTimeStep = 2.0;
	double arrivalRate = 0.33333;
}

namespace randomNumberGenerator
{
	default_random_engine poissonGenerator;
	double lambda = simulationSetting::simulationTimeStep * simulationSetting::arrivalRate / 12.0; 
	poisson_distribution<int> poissonDistribution(lambda);

	random_device rd;
	mt19937 uniformGenerator(rd());
	uniform_int_distribution<int> uniformDistribution1To2(1, 2);
	uniform_int_distribution<int> uniformDistribution1To3(1, 3);
	uniform_int_distribution<int> uniformDistribution1To4(1, 4);
	uniform_int_distribution<int> uniformDistribution1To12(1, 12);

	uniform_real_distribution<> uniformDistribution0To1(0.0, 1.0);
}