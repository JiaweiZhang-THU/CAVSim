#include "cls_random.h"
#include<random>
#include<time.h>

cls_random::cls_random()
{
    srand(time(0));
}

cls_random::~cls_random()
{
}

double cls_random::randomExponential(
    double lambda)
{
    double pV = 0.0;
    while (true)
    {
        pV = (double)rand() / (double)RAND_MAX;
        if (pV != 1)
        {
            break;
        }
    }
    pV = (-1.0 / lambda) * log(1 - pV);
    return pV;
}

double cls_random::randomUniform(
    double dMinValue,
    double dMaxValue)
{
    double pRandomValue = (double)(rand() / (double)RAND_MAX);
    pRandomValue = pRandomValue * (dMaxValue - dMinValue) + dMinValue;
    return pRandomValue;
}

double cls_random::randomGamma(
    double alpha,
    double lambda)
{
    double u, v;
    double x, y;
    double b, c;
    double w, z;
    bool accept = false;

    if (alpha > 1.0)
    {
        b = alpha - 1;
        c = 3 * alpha - 0.75;
        do {
            u = cls_random::randomUniform(0,1);
            v = cls_random::randomUniform(0,1);

            w = u * (1 - u);
            y = sqrt(c / w) * (u - 0.5);
            x = b + y;
            if (x >= 0) {
                z = 64 * w * w * w * v * v;
                double zz = 1 - 2 * y * y / x;
                if (z - zz < 1e-10)
                {
                    accept = true;
                }
                else
                {
                    accept = false;
                }
                if (!accept)
                {
                    double logz = log(z);
                    double zzz = 2 * (b * log(x / b) - y);
                    if (logz - zzz < 1e-10)
                    {
                        accept = true;
                    }
                    else
                    {
                        accept = false;
                    }
                }
            }
        } while (!accept);
        return lambda * x;
    }
    else if (alpha == 1.0)
    {
        return lambda * cls_random::randomExponential(1 / lambda);
    }
    else if (alpha < 1.0)
    {
        double dv = 0.0;
        double b = (exp(1.0) + alpha) / exp(1.0);
        do
        {
            double u1 = cls_random::randomUniform(0,1);
            double p = b * cls_random::randomUniform(0,1);
            double y;
            if (p > 1) {
                y = -log((b - p) / alpha);
                if (u1 < pow(y, alpha - 1)) {
                    dv = y;
                    break;
                }
            }
            else {
                y = pow(p, 1 / alpha);
                if (u1 < exp(-y)) {
                    dv = y;
                    break;
                }
            }
        } while (1);
        return dv * lambda;
    }
    return -1;
}