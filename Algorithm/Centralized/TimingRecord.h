#ifndef _TIMING_RECORD_
#define _TIMING_RECORD_

class TimingRecord
{
public:
	TimingRecord();
	~TimingRecord();

public:
	double UsedTime_Enum;
	double UsedTime_Split;
	double UsedTime_Plan;
	int ConsiderVehicleCount;
	int PermutationNum;
	int AllPlanCount;

public:
	void DeepCopy(TimingRecord& tr);
	TimingRecord(const TimingRecord& tr);

};
#endif // !_TIMING_RECORD_

