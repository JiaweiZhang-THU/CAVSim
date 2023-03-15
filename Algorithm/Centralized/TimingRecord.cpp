#include"TimingRecord.h"


TimingRecord::TimingRecord()
{
}

TimingRecord::~TimingRecord()
{
}

void TimingRecord::DeepCopy(TimingRecord& tr)
{
	tr = *new TimingRecord();
	tr.UsedTime_Enum = TimingRecord::UsedTime_Enum;
	tr.UsedTime_Split = TimingRecord::UsedTime_Split;
	tr.UsedTime_Plan = TimingRecord::UsedTime_Plan;
	tr.ConsiderVehicleCount = TimingRecord::ConsiderVehicleCount;
	tr.PermutationNum = TimingRecord::PermutationNum;
	tr.AllPlanCount = TimingRecord::AllPlanCount;

}

TimingRecord::TimingRecord(const TimingRecord& tr)
{
	TimingRecord::UsedTime_Enum = tr.UsedTime_Enum;
	TimingRecord::UsedTime_Split = tr.UsedTime_Split;
	TimingRecord::UsedTime_Plan = tr.UsedTime_Plan;
	TimingRecord::ConsiderVehicleCount = tr.ConsiderVehicleCount;
	TimingRecord::PermutationNum = tr.PermutationNum;
	TimingRecord::AllPlanCount = tr.AllPlanCount;

}
