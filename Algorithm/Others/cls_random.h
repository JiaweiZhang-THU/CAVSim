#ifndef _CLS_RANDOM_
#define _CLS_RANDOM_
class cls_random
{
public:
	cls_random();
	~cls_random();

public:
	double randomExponential(double lambda);
	double randomUniform(double dMinValue, double dMaxValue);
	double randomGamma(double alpha, double lambda);
};

#endif // !_CLS_RANDOM_

