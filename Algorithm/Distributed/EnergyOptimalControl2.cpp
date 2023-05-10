#include "EnergyOptimalControl2.h"

EnergyOptimalControl2::EnergyOptimalControl2()
{
	EnergyOptimalControl2::TimeStep = 0.1;
	EnergyOptimalControl2::ControlPeriod = 20;
	EnergyOptimalControl2::delta = 6;
}

EnergyOptimalControl2::~EnergyOptimalControl2()
{

}

BPointCoordinate EnergyOptimalControl2::Position2Location(const BPointCoordinate& oldLocation, const double newPosition, const double Angle)
{
	double LocationX = newPosition * cos(Angle) + oldLocation.X;
	double LocationY = newPosition * sin(Angle) + oldLocation.Y;
	return BPointCoordinate(LocationX, LocationY);
}

void EnergyOptimalControl2::Run(DrivingPlan& NewPlan, const Vehicle& aVehicle, const double t_assign)
{
	double t_f = t_assign;
	double OldVelocity = aVehicle.Speed;
	double TargetVelocity = aVehicle.MaxSteerSpeed;
	double Distance = aVehicle.LeftLaneDistance;

	if (t_f <= EnergyOptimalControl2::TimeStep * EnergyOptimalControl2::ControlPeriod)
	{
		RunWithCollisionDetection(NewPlan, aVehicle, t_f, Distance, OldVelocity, TargetVelocity, false);
		return;
	}
	if (RunWithCollisionDetection(NewPlan, aVehicle, t_f, Distance, OldVelocity, TargetVelocity, true) == false)
	{
		NewPlan.Clear();
		Distance = aVehicle.HeadDistance - delta;
		TargetVelocity = aVehicle.HeadSpeed;
		RunWithCollisionDetection(NewPlan, aVehicle, t_f, Distance, OldVelocity, TargetVelocity, false);
		t_f = t_assign - t_f;
		Distance = aVehicle.LeftLaneDistance - Distance;
		OldVelocity = TargetVelocity;
		TargetVelocity = aVehicle.MaxSteerSpeed;
		RunWithCollisionDetection(NewPlan, aVehicle, t_f, Distance, OldVelocity, TargetVelocity, false);
	}
}


bool EnergyOptimalControl2::RunWithCollisionDetection(DrivingPlan& NewPlan, const Vehicle& aVehicle, double& t_assign, double& Distance, const double v_0, const double v_f, const bool detect)
{
	const double D = Distance;
	const double t_f = t_assign;
	const double v_max = aVehicle.MaxStraightSpeed;
	const double v_min = aVehicle.MinStraightSpeed;
	const double a_max = aVehicle.MaxStraightAccel;
	const double a_min = aVehicle.MinStraightAccel;
	const double PosLimit = aVehicle.HeadDistance - EnergyOptimalControl2::delta;
	const double VLimit = aVehicle.HeadSpeed;

	const double Angle = aVehicle.Direction + aVehicle.PoseAngle;
	const double eps = 1e-4;

	double CrtPos = 0;
	double t = 0;
	double v;
	double v_old = v_0;

	

	double minPos;	

	if ((v_0 - v_min) / (-a_min) + (v_f - v_min) / a_max >= t_f)
	{
		double t_vmin = (v_0 + a_max * t_f - v_f) / (a_max - a_min);
		minPos = v_0 * t_vmin + 0.5 * a_min * t_vmin * t_vmin + v_f * (t_f - t_vmin) - 0.5 * a_max * (t_f - t_vmin) * (t_f - t_vmin);
		if (minPos > D - eps)
		{
			double v_realmin = v_0 + a_min * t_vmin;

			while (true) 
			{
				t += EnergyOptimalControl2::TimeStep;
				if (t <= t_vmin)
				{
					v = v_0 + a_min * t;
				}
				else
				{
					v = v_realmin + a_max * (t - t_vmin);
				}
				CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
				v_old = v;
				NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
				if (detect)
					if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
						if (v > VLimit)
						{
							t_assign = t;
							return false;
						}
				NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
				if (CrtPos > D - eps)
				{
					t_assign = t;
					Distance = CrtPos;
					return true;
				}
			}
		}
	}
	else
	{
		minPos = (v_0 * v_0 - v_min * v_min) / (-2 * a_min) + v_min * (t_f - (v_0 - v_min) / (-a_min) - (v_f - v_min) / a_max) + (v_f * v_f - v_min * v_min) / (2 * a_max);
		if (minPos > D - eps)
		{
			double tstart_vmin = v_0 / (-a_min);
			double tend_vmin = t_f - v_f / a_max;

			while (true) 
			{
				t += EnergyOptimalControl2::TimeStep;
				if (t < tstart_vmin)
				{
					v = v_0 + a_min * t;
				}
				else if (t < tend_vmin)
				{
					v = v_min;
				}
				else
				{
					v = v_min + a_max * (t - tend_vmin);
				}
				CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
				v_old = v;
				NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
				if (detect)
					if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
						if (v > VLimit)
						{
							t_assign = t;
							return false;
						}
				NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
				if (CrtPos > D - eps)
				{
					t_assign = t;
					Distance = CrtPos;
					return true;
				}
			}
		}
	}

	double maxPos;	

	if ((v_max - v_0) / a_max + (v_f - v_max) / a_min >= t_f)		
	{
		double t_vmax = (v_f - v_0 - a_min * t_f) / (a_max - a_min);
		maxPos = v_0 * t_vmax + 0.5 * a_max * t_vmax * t_vmax + v_f * (t_f - t_vmax) - 0.5 * a_min * (t_f - t_vmax) * (t_f - t_vmax);
		if (maxPos < D + eps)					
		{
			double v_realmax = sqrt((D + v_0 * v_0 / (2 * a_max) - v_f * v_f / (2 * a_min)) / (0.5 / a_max - 0.5 / a_min));
			if (v_realmax <= v_max)
			{
				t_vmax = (v_realmax - v_0) / a_max;

				while (true) 
				{
					t += EnergyOptimalControl2::TimeStep;
					if (t <= t_vmax)
					{
						v = v_0 + a_max * t;
					}
					else
					{
						v = v_realmax + a_min * (t - t_vmax);
						if (v < v_f)
							v = v_f;
						if (t > t_f + eps && v_f < 0.1)
							return true;
					}
					CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
					v_old = v;
					NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
					if (detect)
						if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
							if (v > VLimit)
							{
								t_assign = t;
								return false;
							}
					NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
					if (CrtPos > D - eps)
					{
						t_assign = t;
						Distance = CrtPos;
						return true;
					}
				} 
			}
			else
			{
				double tstart_vmax = (v_max - v_0) / (a_max);
				double Posstart_vmax = (v_max * v_max - v_0 * v_0) / (2 * a_max);
				double Posend_vmax = D - (v_max * v_max - v_f * v_f) / (-2 * a_min);
				double tend_vmax = tstart_vmax + (Posend_vmax - Posstart_vmax) / v_max;
				while (true) 
				{
					t += EnergyOptimalControl2::TimeStep;
					if (t <= tstart_vmax)
					{
						v = v_0 + a_max * t;
					}
					else if (t <= tend_vmax)
					{
						v = v_max;
					}
					else
					{
						v = v_max + a_min * (t - tend_vmax);
						if (v < v_f)
							v = v_f;
						if (t > t_f + eps && v_f < 0.1)
							return true;
					}
					CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
					v_old = v;
					NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
					if (detect)
						if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
							if (v > VLimit)
							{
								t_assign = t;
								return false;
							}
					NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
					if (CrtPos > D - eps)
					{
						t_assign = t;
						Distance = CrtPos;
						return true;
					}
				}
			}
			return true;
		}
	}
	else
	{
		maxPos = (v_max * v_max - v_0 * v_0) / (2 * a_max) + v_max * (t_f - (v_max - v_0) / a_max - (v_f - v_max) / a_min) + (v_max * v_max - v_f * v_f) / (-2 * a_min);
		if (maxPos < D + eps)
		{
			double tstart_vmax = (v_max - v_0) / (a_max);
			double Posstart_vmax = (v_max * v_max - v_0 * v_0) / (2 * a_max);
			double Posend_vmax = D - (v_max * v_max - v_f * v_f) / (-2 * a_min);
			double tend_vmax = tstart_vmax + (Posend_vmax - Posstart_vmax) / v_max;

			while (true)
			{
				t += EnergyOptimalControl2::TimeStep;
				if (t <= tstart_vmax)
				{
					v = v_0 + a_max * t;
				}
				else if (t <= tend_vmax)
				{
					v = v_max;
				}
				else
				{
					v = v_max + a_min * (t - tend_vmax);
					if (v < v_f)
						v = v_f;
					if (t > t_f + eps && v_f < 0.1)
						return true;
				}
				CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
				v_old = v;
				NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
				if (detect)
					if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
						if (v > VLimit)
						{
							t_assign = t;
							return false;
						}
				NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
				if (CrtPos > D - eps)
				{
					t_assign = t;
					Distance = CrtPos;
					return true;
				}
			}
			return true;
		}
	}

	double C1 = 6 * (t_f * (v_f + v_0) - 2 * D) / (t_f * t_f * t_f);
	double C2 = 2 * (t_f * (v_f + 2 * v_0) - 3 * D) / (t_f * t_f);
	double C3 = v_0;

	if (abs(C1) < eps)
	{
		double Acc = fmin(fmax(-C2, a_min), a_max);

		while (true)
		{
			t += EnergyOptimalControl2::TimeStep;
			v = v_0 + Acc * t;
			if (t > t_f + eps)
			{
				if (v_f < 0.1)
					return true;
				v = v_f;
			}
			CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
			v_old = v;
			NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
			if (detect)
				if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
					if (v > VLimit)
					{
						t_assign = t;
						return false;
					}
			NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
			if (CrtPos > D - eps)
			{
				Distance = CrtPos;
				return true;
			}
		}
	}
	double v_extrme = v_0 - 0.5 * C2 * C2 / C1;
	double t_extrme = C2 / C1;
	if (-C2 >= a_min && -C2 <= a_max && C1 * t_f - C2 >= a_min && C1 * t_f - C2 <= a_max)
	{
		if (t_extrme <= 0 || t_extrme >= t_f || (v_extrme >= v_min && v_extrme <= v_max))
		{
			while (true)
			{
				t += EnergyOptimalControl2::TimeStep;
				v = C1 / 2 * t * t - C2 * t + C3;
				if (t > t_f + eps)
				{
					if (v_f < 0.1)
						return true;
					v = v_f;
				}
				CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
				v_old = v;
				NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
				if (detect)
					if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
						if (v > VLimit)
						{
							t_assign = t;
							return false;
						}
				NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
				if (CrtPos > D - eps)
				{
					Distance = CrtPos;
					return true;
				}
			}
		}
		else																		
		{
			double beta1, beta2, A, t1, t2;
			if (v_extrme > v_max)															
			{
				beta1 = sqrt(2 * (v_max - v_0));
				beta2 = sqrt(2 * (v_max - v_f));
				A = (beta1 * beta1 * beta1 + beta2 * beta2 * beta2) / (6 * (v_max * t_f - D));
				t1 = beta1 / A;
				t2 = t_f - beta2 / A;
				if (beta1 * A <= a_max && -beta2 * A >= a_min)								
				{
					while (true)
					{
						t += EnergyOptimalControl2::TimeStep;
						if (t <= t1)
						{
							v = -A * A / 2 * t * t + beta1 * A * t + v_0;
						}
						else if (t <= t2)
						{
							v = v_max;
						}
						else if (t <= t_f + eps)
						{
							v = -A * A / 2 * t * t + (A * A * t_f - beta2 * A) * t + v_f - A * A * t_f * t_f / 2 + beta2 * A * t_f;
						}
						else
						{
							if (v_f < 0.1)
								return true;
							v = v_f;
						}
						CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
						v_old = v;
						NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
						if (detect)
							if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
								if (v > VLimit)
								{
									t_assign = t;
									return false;
								}
						NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
						if (CrtPos > D - eps)
						{
							Distance = CrtPos;
							return true;
						}
					}
				}
			}
			else													
			{
				beta1 = sqrt(2 * (v_0 - v_min));
				beta2 = sqrt(2 * (v_f - v_min));
				A = (beta1 * beta1 * beta1 + beta2 * beta2 * beta2) / (6 * (D - v_min * t_f));
				t1 = beta1 / A;
				t2 = t_f - beta2 / A;
				if (-beta1 * A >= a_min && beta2 * A <= a_max)						
				{
					while (true)
					{
						t += EnergyOptimalControl2::TimeStep;
						if (t <= t1)
						{
							v = A * A / 2 * t * t - beta1 * A * t + v_0;
						}
						else if (t <= t2)
						{
							v = v_min;
						}
						else if (t <= t_f + eps)
						{
							v = A * A / 2 * t * t - (A * A * t_f - beta2 * A) * t + v_f + A * A * t_f * t_f / 2 - beta2 * A * t_f;
						}
						else
						{
							if (v_f < 0.1)
								return true;
							v = v_f;
						}
						CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
						v_old = v;
						NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
						if (detect)
							if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
								if (v > VLimit)
								{
									t_assign = t;
									return false;
								}
						NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
						if (CrtPos > D - eps)
						{
							Distance = CrtPos;
							return true;
						}
					}
				}
			}
		}
	}
	else if ((v_extrme <= v_max && v_extrme >= v_min) || t_extrme <= 0 || t_extrme >= t_f)	
	{
		double t1, t2;
		double a1, a2;	
		bool Solved = false;
		bool Vactive = false;
		if (C1 > 0)			
		{
			a1 = a_min;
			a2 = a_max;
		}
		else
		{
			a1 = a_max;
			a2 = a_min;
		}
		double A, B, C;
		double sum;	
		double pro;	
		double dif;
		double v_0_ = v_0;
		double v_f_ = v_f;

		for (unsigned int i = 0; i < 3; i++)
		{
			if (i == 0)								
			{
			}
			else if (i == 1)						
			{
				if (C1 * t_f - C2 < a_max && C1 * t_f - C2 > a_min)
					continue;
				a1 = (4 * pow(v_f_ - v_0_ - a2 * t_f, 2)) / (6 * v_f_ * t_f - 3 * a2 * t_f * t_f - 6 * D) + a2;
			}
			else									
			{
				double temp;
				temp = a1;
				a1 = -a2;
				a2 = -temp;
				v_0_ = v_f;
				v_f_ = v_0;
				a1 = (4 * pow(v_f_ - v_0_ - a2 * t_f, 2)) / (6 * v_f_ * t_f - 3 * a2 * t_f * t_f - 6 * D) + a2;
			}

			sum = 2 / (a2 - a1) * (v_0_ - v_f_ + a2 * t_f);
			pro = (D + sum * (v_f_ - v_0_ - a2 * t_f) - v_f_ * t_f + a2 * t_f * t_f / 2) * 6 / (a1 - a2) - 2 * sum * sum;
			if (pro >= -eps || sum * sum - 4 * pro >= -eps)		
			{
				dif = -sqrt(fmax(sum * sum - 4 * pro, 0));
				t1 = (sum + dif) / 2;
				t2 = (sum - dif) / 2;
				if (abs(t1) < eps)
					t1 = 0;
				if (abs(t2) < eps)
					t2 = 0;
				if (abs(t1 - t_f) < eps)
					t1 = t_f;
				if (abs(t2 - t_f) < eps)
					t2 = t_f;
				if (t2 <= t_f && t1 >= 0)						
				{
					if (t1 + 1e-4 > t2)							
					{
						A = 1;
						B = -2 * t1;
						C = v_0_ + a1 * t1 - t1 * t1;			
						t2 = t1;
					}
					else
					{
						A = (a1 - a2) / dif / 2;
						B = (a1 * t2 - a2 * t1) / (-dif);
						C = ((a1 - a2) * pro / 2 + v_0_ * t2 - v_f_ * t1 + a2 * t_f * t1) / (-dif);
					}
					v_extrme = C - B * B / (4 * A);
					if (v_extrme >= v_min && v_extrme <= v_max)
					{
						Solved = true;
						if (i == 2)								
						{
							double temp;
							temp = a1;
							a1 = -a2;
							a2 = -temp;
							temp = t1;
							t1 = t_f - t2;
							t2 = t_f - temp;
							C = A * t_f * t_f + B * t_f + C;
							B = -B - 2 * A * t_f;
						}
					}
					break;
				}
			}
		}
		if (Solved)
		{
			double v_t2 = A * t2 * t2 + B * t2 + C;
			while (true)
			{
				t += EnergyOptimalControl2::TimeStep;
				if (t <= t1)
				{
					v = v_0 + a1 * t;
				}
				else if (t <= t2)
				{
					v = A * t * t + B * t + C;
				}
				else if (t <= t_f + eps)
				{
					v = v_t2 + a2 * (t - t2);
				}
				else
				{
					if (v_f < 0.1)
						return true;
					v = v_f;
				}
				CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
				v_old = v;
				NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
				if (detect)
					if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
						if (v > VLimit)
						{
							t_assign = t;
							return false;
						}
				NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
				if (CrtPos > D - eps)
				{
					Distance = CrtPos;
					return true;
				}
			}
		}
	}

	if (C2 > 0)						
	{
		double Acc = (pow(v_0 - v_min, 2) + pow(v_f - v_min, 2)) / 2 / (D - v_min * t_f);
		double t1, t2;
		if (Acc <= fmin(a_max, -a_min))
		{
			t1 = (v_0 - v_min) / Acc;
			t2 = t_f - (v_f - v_min) / Acc;
			while (true)
			{
				t += EnergyOptimalControl2::TimeStep;
				if (t <= t1)
				{
					v = v_0 - Acc * t;
				}
				else if (t <= t2)
				{
					v = v_min;
				}
				else if (t <= t_f + eps)
				{
					v = v_min + Acc * (t - t2);
				}
				else
				{
					if (v_f < 0.1)
						return true;
					v = v_f;
				}
				CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
				v_old = v;
				NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
				if (detect)
					if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
						if (v > VLimit)
						{
							t_assign = t;
							return false;
						}
				NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
				if (CrtPos > D - eps)
				{
					Distance = CrtPos;
					return true;
				}
			}
		}
		else
		{
			if (a_max > -a_min)
			{
				Acc = pow(v_f - v_min, 2) / (2 * D + pow(v_0 - v_min, 2) / a_min - 2 * v_min * t_f);
				t1 = (v_0 - v_min) / (-a_min);
				t2 = t_f - (v_f - v_min) / Acc;
				while (true)
				{
					t += EnergyOptimalControl2::TimeStep;
					if (t <= t1)
					{
						v = v_0 + a_min * t;
					}
					else if (t <= t2)
					{
						v = v_min;
					}
					else if (t <= t_f + eps)
					{
						v = v_min + Acc * (t - t2);
					}
					else
					{
						if (v_f < 0.1)
							return true;
						v = v_f;
					}
					CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
					v_old = v;
					NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
					if (detect)
						if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
							if (v > VLimit)
							{
								t_assign = t;
								return false;
							}
					NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
					if (CrtPos > D - eps)
					{
						Distance = CrtPos;
						return true;
					}
				}
			}
			else
			{
				Acc = pow(v_0 - v_min, 2) / (2 * D - pow(v_f - v_min, 2) / a_max - 2 * v_min * t_f);
				t1 = (v_0 - v_min) / Acc;
				t2 = t_f - (v_f - v_min) / a_max;
				while (true)
				{
					t += EnergyOptimalControl2::TimeStep;
					if (t <= t1)
					{
						v = v_0 - Acc * t;
					}
					else if (t <= t2)
					{
						v = v_min;
					}
					else if (t <= t_f + eps)
					{
						v = v_min + a_max * (t - t2);
					}
					else
					{
						if (v_f < 0.1)
							return true;
						v = v_f;
					}
					CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
					v_old = v;
					NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
					if (detect)
						if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
							if (v > VLimit)
							{
								t_assign = t;
								return false;
							}
					NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
					if (CrtPos > D - eps)
					{
						Distance = CrtPos;
						return true;
					}
				}
			}
		}
	}
	else
	{
		double Acc = (pow(v_0 - v_max, 2) + pow(v_f - v_max, 2)) / 2 / (v_max * t_f - D);
		double t1, t2;
		if (Acc <= fmin(a_max, -a_min))
		{
			t1 = (v_max - v_0) / Acc;
			t2 = t_f - (v_max - v_f) / Acc;
			while (true)
			{
				t += EnergyOptimalControl2::TimeStep;
				if (t <= t1)
				{
					v = v_0 + Acc * t;
				}
				else if (t <= t2)
				{
					v = v_max;
				}
				else if (t <= t_f + eps)
				{
					v = v_max - Acc * (t - t2);
				}
				else
				{
					if (v_f < 0.1)
						return true;
					v = v_f;
				}
				CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
				v_old = v;
				NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
				if (detect)
					if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
						if (v > VLimit)
						{
							t_assign = t;
							return false;
						}
				NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
				if (CrtPos > D - eps)
				{
					Distance = CrtPos;
					return true;
				}
			}
		}
		else
		{
			if (a_max > -a_min)	
			{
				Acc = pow(v_0 - v_max, 2) / (-2 * D + pow(v_f - v_max, 2) / a_min + 2 * v_max * t_f);
				t1 = (v_max - v_0) / Acc;
				t2 = t_f - (v_f - v_max) / a_min;
				while (true)
				{
					t += EnergyOptimalControl2::TimeStep;
					if (t <= t1)
					{
						v = v_0 + Acc * t;
					}
					else if (t <= t2)
					{
						v = v_max;
					}
					else if (t <= t_f + eps)
					{
						v = v_max + a_min * (t - t2);
					}
					else
					{
						if (v_f < 0.1)
							return true;
						v = v_f;
					}
					CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
					v_old = v;
					NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
					if (detect)
						if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
							if (v > VLimit)
							{
								t_assign = t;
								return false;
							}
					NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
					if (CrtPos > D - eps)
					{
						Distance = CrtPos;
						return true;
					}
				}
			}
			else
			{
				Acc = pow(v_f - v_max, 2) / (-2 * D - pow(v_0 - v_max, 2) / a_max + 2 * v_max * t_f);
				t1 = (v_max - v_0) / a_max;
				t2 = t_f - (v_max - v_f) / Acc;
				while (true)
				{
					t += EnergyOptimalControl2::TimeStep;
					if (t <= t1)
					{
						v = v_0 + a_max * t;
					}
					else if (t <= t2)
					{
						v = v_max;
					}
					else if (t <= t_f + eps)
					{
						v = v_max - Acc * (t - t2);
					}
					else
					{
						if (v_f < 0.1)
							return true;
						v = v_f;
					}
					CrtPos += (v_old + v) / 2 * EnergyOptimalControl2::TimeStep;
					v_old = v;
					NewPlan.Add(Position2Location(aVehicle.Location, CrtPos, Angle), Angle, v, 0);
					if (detect)
						if ((CrtPos >= PosLimit && CrtPos - v * EnergyOptimalControl2::TimeStep <= PosLimit) || PosLimit < eps)
							if (v > VLimit)
							{
								t_assign = t;
								return false;
							}
					NewPlan.DistanceList.push_back(D - CrtPos + aVehicle.LeftInterDistance);
					if (CrtPos > D - eps)
					{
						Distance = CrtPos;
						return true;
					}
				}
			}
		}
	}

}
