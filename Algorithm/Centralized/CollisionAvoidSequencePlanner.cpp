#include "CollisionAvoidSequencePlanner.h"

CollisionAvoidSequencePlanner::CollisionAvoidSequencePlanner()
{
}

CollisionAvoidSequencePlanner::CollisionAvoidSequencePlanner(int aIntersectionOneWayLanesNum, double aLaneWidth, BPointCoordinate aIntersectionCenter, map<int, Vehicle> aSimuVehicleDict, list<list<list<int>>> aPlanBasedApproach_AllSeqList)
{
    CollisionAvoidSequencePlanner::IntersectionOneWayLanesNum = aIntersectionOneWayLanesNum;
    CollisionAvoidSequencePlanner::LaneWidth = aLaneWidth;
    CollisionAvoidSequencePlanner::IntersectionCenter = aIntersectionCenter;
    CollisionAvoidSequencePlanner::MyCollisionDetection = *new CollisionDetection(CollisionAvoidSequencePlanner::IntersectionOneWayLanesNum, CollisionAvoidSequencePlanner::LaneWidth, CollisionAvoidSequencePlanner::IntersectionCenter);

    for (map<int, Vehicle>::iterator VIter = aSimuVehicleDict.begin(); VIter != aSimuVehicleDict.end(); VIter++)
    {
        CollisionAvoidSequencePlanner::SimuVehicleDict[VIter->first] = VIter->second;
    }

    for (list<list<list<int>>>::iterator intLLLIter = aPlanBasedApproach_AllSeqList.begin(); intLLLIter != aPlanBasedApproach_AllSeqList.end(); intLLLIter++)
    {
        list<list<int>> intLList = *new list<list<int>>();
        for (list<list<int>>::iterator intLLIter = intLLLIter->begin(); intLLIter != intLLLIter->end(); intLLIter++)
        {
            list<int> intList = *new list<int>();
            for (list<int>::iterator intLIter = intLLIter->begin(); intLIter != intLLIter->end(); intLIter)
            {
                intList.push_back(*intLIter);
            }
            intLList.push_back(intList);
        }
        CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.push_back(intLList);
    }

}
CollisionAvoidSequencePlanner::CollisionAvoidSequencePlanner(list<int> rawSeqList, int aIntersectionOneWayLanesNum, double aLaneWidth, BPointCoordinate aIntersectionCenter, map<int, Vehicle> aSimuVehicleDict, list<list<list<int>>> aPlanBasedApproach_AllSeqList)
{
    CollisionAvoidSequencePlanner::IntersectionOneWayLanesNum = aIntersectionOneWayLanesNum;
    CollisionAvoidSequencePlanner::LaneWidth = aLaneWidth;
    CollisionAvoidSequencePlanner::IntersectionCenter = aIntersectionCenter;
    CollisionAvoidSequencePlanner::MyCollisionDetection = *new CollisionDetection(CollisionAvoidSequencePlanner::IntersectionOneWayLanesNum, CollisionAvoidSequencePlanner::LaneWidth, CollisionAvoidSequencePlanner::IntersectionCenter);

    for (map<int, Vehicle>::iterator VIter = aSimuVehicleDict.begin(); VIter != aSimuVehicleDict.end(); VIter++)
    {
        CollisionAvoidSequencePlanner::SimuVehicleDict[VIter->first] = VIter->second;
    }

    for (list<list<list<int>>>::iterator intLLLIter = aPlanBasedApproach_AllSeqList.begin(); intLLLIter != aPlanBasedApproach_AllSeqList.end(); intLLLIter++)
    {
        list<list<int>> intLList = *new list<list<int>>();
        for (list<list<int>>::iterator intLLIter = intLLLIter->begin(); intLLIter != intLLLIter->end(); intLLIter++)
        {
            list<int> intList = *new list<int>();
            for (list<int>::iterator intLIter = intLLIter->begin(); intLIter != intLLIter->end(); intLIter)
            {
                intList.push_back(*intLIter);
            }
            intLList.push_back(intList);
        }
        CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.push_back(intLList);
    }

    int i = 0;
    for (list<int>::iterator IIter = rawSeqList.begin(); IIter != rawSeqList.end(); IIter++, i++)
    {
        CollisionAvoidSequencePlanner::BasicSeqArray[i] = *IIter;
    }

    int localMin = INT32_MAX - 1;
    int localMax = -1;
    int seqCount = int(rawSeqList.size());

    for (int i = 0; i < seqCount; i++)
    {
        if (CollisionAvoidSequencePlanner::BasicSeqArray[i] < localMin)
        {
            localMin = CollisionAvoidSequencePlanner::BasicSeqArray[i];
        }
        if (CollisionAvoidSequencePlanner::BasicSeqArray[i] > localMax)
        {
            localMax = CollisionAvoidSequencePlanner::BasicSeqArray[i];
        }
    }

    CollisionAvoidSequencePlanner::_offset = localMin;

    int recordCount = localMax - localMin;
    CollisionAvoidSequencePlanner::_find = new int[recordCount];

    for (int i = 0; i < seqCount; i++)
    {
        CollisionAvoidSequencePlanner::_find[CollisionAvoidSequencePlanner::BasicSeqArray[i] - CollisionAvoidSequencePlanner::_offset] = i;
    }

    CollisionAvoidSequencePlanner::_next = new int[recordCount];
    CollisionAvoidSequencePlanner::_prev = new int[recordCount];

    for (int i = 0; i < recordCount; i++)
    {
        CollisionAvoidSequencePlanner::_next[i] = -1;
        CollisionAvoidSequencePlanner::_prev[i] = -1;
    }

    for (int i = 0; i < seqCount - 1; i++)
    {
        CollisionAvoidSequencePlanner::_cut[i] = false;
    }

    CollisionAvoidSequencePlanner::BinarayPruneList = *new list<int>();
}

CollisionAvoidSequencePlanner::~CollisionAvoidSequencePlanner()
{
}

void CollisionAvoidSequencePlanner::Init()
{
    CollisionAvoidSequencePlanner::CollisionPairList = *new list<CollisionPair>();
    CollisionAvoidSequencePlanner::FinalSeqList = *new list<list<int>>();
    CollisionAvoidSequencePlanner::recordStack = *new stack<int>();

}

void CollisionAvoidSequencePlanner::ResetSindex()
{
    CollisionAvoidSequencePlanner::sindex = 0;
    CollisionAvoidSequencePlanner::CutThreshold = INT32_MAX;
}
void CollisionAvoidSequencePlanner::Load(list<int> rawSeqList, list<list<list<int>>> aPlanBasedApproach_AllSeqList)
{
    for (list<list<list<int>>>::iterator intLLLIter = aPlanBasedApproach_AllSeqList.begin(); intLLLIter != aPlanBasedApproach_AllSeqList.end(); intLLLIter++)
    {
        list<list<int>> intLList = *new list<list<int>>();
        for (list<list<int>>::iterator intLLIter = intLLLIter->begin(); intLLIter != intLLLIter->end(); intLLIter++)
        {
            list<int> intList = *new list<int>();
            for (list<int>::iterator intLIter = intLLIter->begin(); intLIter != intLLIter->end(); intLIter)
            {
                intList.push_back(*intLIter);
            }
            intLList.push_back(intList);
        }
        CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.push_back(intLList);
    }

    int i = 0;
    for (list<int>::iterator IIter = rawSeqList.begin(); IIter != rawSeqList.end(); IIter++,i++)
    {
        CollisionAvoidSequencePlanner::BasicSeqArray[i] = *IIter;
    }

    int localMin = INT32_MAX - 1;
    int localMax = -1;
    int seqCount = int(rawSeqList.size());

    for (int i = 0; i < seqCount; i++)
    {
        if (CollisionAvoidSequencePlanner::BasicSeqArray[i] < localMin)
        {
            localMin = CollisionAvoidSequencePlanner::BasicSeqArray[i];
        }
        if (CollisionAvoidSequencePlanner::BasicSeqArray[i] > localMax)
        {
            localMax = CollisionAvoidSequencePlanner::BasicSeqArray[i];
        }
    }

    CollisionAvoidSequencePlanner::_offset = localMin;

    int recordCount = localMax - localMin + 1;
    CollisionAvoidSequencePlanner::_find = new int[recordCount];

    for (int i = 0; i < seqCount; i++)
    {
        CollisionAvoidSequencePlanner::_find[CollisionAvoidSequencePlanner::BasicSeqArray[i] - CollisionAvoidSequencePlanner::_offset] = i;
    }

    CollisionAvoidSequencePlanner::_next = new int[recordCount];
    CollisionAvoidSequencePlanner::_prev = new int[recordCount];

    for (int i = 0; i < recordCount; i++)
    {
        CollisionAvoidSequencePlanner::_next[i] = -1;
        CollisionAvoidSequencePlanner::_prev[i] = -1;
    }

    for (int i = 0; i < seqCount - 1; i++)
    {
        CollisionAvoidSequencePlanner::_cut[i] = false;
    }

    CollisionAvoidSequencePlanner::BinarayPruneList = *new list<int>();
}

void CollisionAvoidSequencePlanner::Run()
{
    CollisionAvoidSequencePlanner::FirstCheckNeighborCollision();
    CollisionAvoidSequencePlanner::FindRestCollisionPairs();
    CollisionAvoidSequencePlanner::PruneTreeAlgor();
}

void CollisionAvoidSequencePlanner::FirstCheckNeighborCollision()
{
    for (int i = 0; i < CollisionAvoidSequencePlanner::BasicSeqArray.size() - 1; i++)
    {
        Vehicle VehicleA = CollisionAvoidSequencePlanner::SimuVehicleDict[CollisionAvoidSequencePlanner::BasicSeqArray[i]];
        Vehicle VehicleB = CollisionAvoidSequencePlanner::SimuVehicleDict[CollisionAvoidSequencePlanner::BasicSeqArray[i+1]];
        if (CollisionAvoidSequencePlanner::MyCollisionDetection.IsTwoVehicleCollision(VehicleA, VehicleB) == true)
        {
            CollisionAvoidSequencePlanner::UpdateNextAndPrevByAddCut(i);
        }
    }
}

void CollisionAvoidSequencePlanner::FindRestCollisionPairs()
{
    CollisionAvoidSequencePlanner::CollisionPairList.clear();
    for (int i = 0; i < CollisionAvoidSequencePlanner::BasicSeqArray.size(); i++)
    {
        int currId = CollisionAvoidSequencePlanner::BasicSeqArray[i];
        Vehicle VehicleA = CollisionAvoidSequencePlanner::SimuVehicleDict[currId];
        int endId;
        if (CollisionAvoidSequencePlanner::_next[currId - CollisionAvoidSequencePlanner::_offset] == -1)
        {
            endId = CollisionAvoidSequencePlanner::BasicSeqArray[int(CollisionAvoidSequencePlanner::BasicSeqArray.size()) - 1];
        }
        else
        {
            endId = CollisionAvoidSequencePlanner::_prev[_next[currId - CollisionAvoidSequencePlanner::_offset] - CollisionAvoidSequencePlanner::_offset];
        }
        for (int j = i + 1; j < CollisionAvoidSequencePlanner::_find[endId - CollisionAvoidSequencePlanner::_offset]; j++)
        {
            Vehicle VehicleB = CollisionAvoidSequencePlanner::SimuVehicleDict[CollisionAvoidSequencePlanner::BasicSeqArray[j]];
            if (CollisionAvoidSequencePlanner::MyCollisionDetection.IsTwoVehicleCollision(VehicleA, VehicleB) == true)
            {
                CollisionPair cp = *new CollisionPair();
                cp.A = CollisionAvoidSequencePlanner::BasicSeqArray[i];
                cp.B = CollisionAvoidSequencePlanner::BasicSeqArray[j];
                CollisionAvoidSequencePlanner::CollisionPairList.push_back(cp);
            }
        }
    }
}

list<CollisionAvoidSequencePlanner::CollisionPair> CollisionAvoidSequencePlanner::PruneCollisionPairs(int index)
{
    list<CollisionPair> deleList = *new list<CollisionPair>();
    for (list<CollisionPair>::iterator CIter = CollisionAvoidSequencePlanner::CollisionPairList.begin(); CIter != CollisionAvoidSequencePlanner::CollisionPairList.end();)
    {
        if (CollisionAvoidSequencePlanner::_find[CIter->B - CollisionAvoidSequencePlanner::_offset] <= index)
        {
            CIter++;
            continue;
        }
        else 
        {
            if (CollisionAvoidSequencePlanner::_find[CIter->A - CollisionAvoidSequencePlanner::_offset] <= index)
            {
                deleList.push_back(*CIter);
                CIter = CollisionAvoidSequencePlanner::CollisionPairList.erase(CIter);
            }
            else
            {
                CIter++;
            }
        }
    }
   

    return deleList;
}

void CollisionAvoidSequencePlanner::PruneTreeAlgor()
{
    CollisionAvoidSequencePlanner::InteriorAlgorLoop(CollisionAvoidSequencePlanner::CountInitialCutNum());

    CollisionAvoidSequencePlanner::RemoveThoseCutTooMuch();
    CollisionAvoidSequencePlanner::RemoveThoseIsAnotherPermutation();
}

int CollisionAvoidSequencePlanner::CountInitialCutNum()
{
    int num = 0;
    for (int i = 0; i < CollisionAvoidSequencePlanner::BasicSeqArray.size() - 1; i++)
    {
        if (CollisionAvoidSequencePlanner::_cut[i] == true)
        {
            num += 1;
        }
    }
    return num;
}

void CollisionAvoidSequencePlanner::InteriorAlgorLoop(int cutNum)
{
    if (CollisionAvoidSequencePlanner::CollisionPairList.size() == 0)//没有碰撞对了，就应该输出了
    {
        if (CollisionAvoidSequencePlanner::isRedundant() == false)
        {
            if (cutNum < CollisionAvoidSequencePlanner::CutThreshold)
            {
                CollisionAvoidSequencePlanner::CutThreshold = cutNum;
            }

            CollisionAvoidSequencePlanner::GenerateFinalOrder();
            CollisionAvoidSequencePlanner::AddToTheAllSeqList();
        }
    }
    else
    {
        if (cutNum < CollisionAvoidSequencePlanner::CutThreshold)
        {
            CollisionPair firstCp = CollisionAvoidSequencePlanner::CollisionPairList.front();
            int leftIndex = CollisionAvoidSequencePlanner::_find[firstCp.A - _offset];
            int rightIndex = CollisionAvoidSequencePlanner::_find[firstCp.B - _offset];
            for (int i = leftIndex; i < rightIndex; i++)
            {
                UpdateNextAndPrevByAddCut(i);
                list<CollisionPair> bakCpList = PruneCollisionPairs(i);
                CollisionAvoidSequencePlanner::recordStack.push(i);
                CollisionAvoidSequencePlanner::InteriorAlgorLoop(cutNum + 1);

                for (list<CollisionPair>::reverse_iterator CIterR = bakCpList.rbegin(); CIterR != bakCpList.rend(); CIterR++)
                {
                    CollisionAvoidSequencePlanner::CollisionPairList.push_front(*CIterR);

                }
                
                CollisionAvoidSequencePlanner::recordStack.pop();
                CollisionAvoidSequencePlanner::UpdateNextAndPrevByDelCut(i);
            }
        }
    }
}

void CollisionAvoidSequencePlanner::RemoveThoseCutTooMuch()
{
    for (list<list<list<int>>>::iterator intLLLIter = CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.begin(); intLLLIter != CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.end();)
    {
        if (intLLLIter->size() - 1 > CollisionAvoidSequencePlanner::CutThreshold)
        {
            intLLLIter = CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.erase(intLLLIter);

            CollisionAvoidSequencePlanner::sindex--;
        }
        else
        {
            intLLLIter++;
        }
    }
}

void CollisionAvoidSequencePlanner::RemoveThoseIsAnotherPermutation()
{
}

void CollisionAvoidSequencePlanner::UpdateNextAndPrevByAddCut(int index)
{
    CollisionAvoidSequencePlanner::_cut[index] = true;
    int leftId = CollisionAvoidSequencePlanner::BasicSeqArray[index];
    int leftEndId = CollisionAvoidSequencePlanner::_prev[leftId - CollisionAvoidSequencePlanner::_offset];
    int rightId = CollisionAvoidSequencePlanner::BasicSeqArray[index + 1];
    int rightStartId = CollisionAvoidSequencePlanner::_next[rightId - CollisionAvoidSequencePlanner::_offset];

    int thisStartId;
    int thisEndId;

    if (leftEndId != -1)
    {
        thisStartId = CollisionAvoidSequencePlanner::_next[leftEndId - _offset];
    }
    else
    {
        thisStartId = CollisionAvoidSequencePlanner::BasicSeqArray[0];
    }

    if (rightStartId != -1)
    {
        thisEndId = CollisionAvoidSequencePlanner::_prev[rightStartId - _offset];
    }
    else
    {
        thisEndId = CollisionAvoidSequencePlanner::BasicSeqArray[int(CollisionAvoidSequencePlanner::BasicSeqArray.size()) - 1];
    }

    for (int i = CollisionAvoidSequencePlanner::_find[thisStartId - CollisionAvoidSequencePlanner::_offset]; i <= CollisionAvoidSequencePlanner::_find[leftId - CollisionAvoidSequencePlanner::_offset]; i++)
    {
        CollisionAvoidSequencePlanner::_next[CollisionAvoidSequencePlanner::BasicSeqArray[i] - CollisionAvoidSequencePlanner::_offset] = rightId;
    }

    for (int i = CollisionAvoidSequencePlanner::_find[rightId - CollisionAvoidSequencePlanner::_offset]; i <= CollisionAvoidSequencePlanner::_find[thisEndId - CollisionAvoidSequencePlanner::_offset]; i++)
    {
        CollisionAvoidSequencePlanner::_prev[CollisionAvoidSequencePlanner::BasicSeqArray[i] - CollisionAvoidSequencePlanner::_offset] = leftId;
    }
}

void CollisionAvoidSequencePlanner::UpdateNextAndPrevByDelCut(int index)
{
    CollisionAvoidSequencePlanner::_cut[index] = false;
    int leftId = CollisionAvoidSequencePlanner::BasicSeqArray[index];
    int leftEndId = CollisionAvoidSequencePlanner::_prev[leftId - CollisionAvoidSequencePlanner::_offset];
    int rightId = CollisionAvoidSequencePlanner::BasicSeqArray[index + 1];
    int rightStartId = CollisionAvoidSequencePlanner::_next[rightId - CollisionAvoidSequencePlanner::_offset];

    int thisStartId;
    int thisEndId;

    if (leftEndId != -1)
    {
        thisStartId = CollisionAvoidSequencePlanner::_next[leftEndId - CollisionAvoidSequencePlanner::_offset];
    }
    else
    {
        thisStartId = CollisionAvoidSequencePlanner::BasicSeqArray[0];
    }

    if (rightStartId != -1)
    {
        thisEndId = CollisionAvoidSequencePlanner::_prev[rightStartId - _offset];
    }
    else
    {
        thisEndId = CollisionAvoidSequencePlanner::BasicSeqArray[int(CollisionAvoidSequencePlanner::BasicSeqArray.size()) - 1];
    }


    for (int i = CollisionAvoidSequencePlanner::_find[thisStartId - CollisionAvoidSequencePlanner::_offset]; i <= CollisionAvoidSequencePlanner::_find[leftId - CollisionAvoidSequencePlanner::_offset]; i++)
    {
        CollisionAvoidSequencePlanner::_next[CollisionAvoidSequencePlanner::BasicSeqArray[i] - CollisionAvoidSequencePlanner::_offset] = CollisionAvoidSequencePlanner::_next[CollisionAvoidSequencePlanner::_next[CollisionAvoidSequencePlanner::BasicSeqArray[i] - CollisionAvoidSequencePlanner::_offset] - CollisionAvoidSequencePlanner::_offset];
    }

    for (int i = CollisionAvoidSequencePlanner::_find[rightId - CollisionAvoidSequencePlanner::_offset]; i <= CollisionAvoidSequencePlanner::_find[thisEndId - CollisionAvoidSequencePlanner::_offset]; i++)
    {
        CollisionAvoidSequencePlanner::_prev[CollisionAvoidSequencePlanner::BasicSeqArray[i] - CollisionAvoidSequencePlanner::_offset] = CollisionAvoidSequencePlanner::_prev[CollisionAvoidSequencePlanner::_prev[CollisionAvoidSequencePlanner::BasicSeqArray[i] - CollisionAvoidSequencePlanner::_offset] - CollisionAvoidSequencePlanner::_offset];
    }
}

void CollisionAvoidSequencePlanner::GenerateFinalOrder()
{
    CollisionAvoidSequencePlanner::FinalSeqList = *new list<list<int>>();
    list<int> subList = *new list<int>();
    FinalSeqList.push_back(subList);
    
    for (int i = 0; i < CollisionAvoidSequencePlanner::BasicSeqArray.size(); i++)
    {
        list<list<int>>::iterator IntLLIter = CollisionAvoidSequencePlanner::FinalSeqList.end();
        IntLLIter--;
        IntLLIter->push_back(CollisionAvoidSequencePlanner::BasicSeqArray[i]);


        if (CollisionAvoidSequencePlanner::_cut[i])
        {
            subList = *new list<int>();
            CollisionAvoidSequencePlanner::FinalSeqList.push_back(subList);
        }
    }

}

void CollisionAvoidSequencePlanner::AddToTheAllSeqList()
{
    CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.push_back(CollisionAvoidSequencePlanner::FinalSeqList);
}

void CollisionAvoidSequencePlanner::AddToTheAllSeqListCite(list<list<list<int>>>& aAllSeqList)
{
    aAllSeqList = CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList;
}


int CollisionAvoidSequencePlanner::generateCutBinary()
{
    int sum = 0;
   for (int i = 0; i < CollisionAvoidSequencePlanner::BasicSeqArray.size(); i++)
    {
        sum = sum * 2;
        sum += (CollisionAvoidSequencePlanner::_cut[i]) ? 1 : 0;

    }
    return sum;
}


bool CollisionAvoidSequencePlanner::isRedundant()
{
    int currBinCut = CollisionAvoidSequencePlanner::generateCutBinary();
    list<int>::iterator IntIter = CollisionAvoidSequencePlanner::BinarayPruneList.begin();
    for (int i = 0; i < CollisionAvoidSequencePlanner::BinarayPruneList.size();)
    {

        int existBinCut = *IntIter;
        int X = currBinCut ^ existBinCut;
        int B = ~currBinCut;
        int result = X & B;
        if (result == 0)
        {
            return true;
        }
        else
        {
            int C = ~existBinCut;
            int anotherResult = X & C;
            if (anotherResult == 0)
            {
                list<list<list<int>>>::iterator IntLLLIter = CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.begin();
                advance(IntLLLIter, sindex - (BinarayPruneList.size() - i));
                CollisionAvoidSequencePlanner::PlanBasedApproach_AllSeqList.erase(IntLLLIter);

                CollisionAvoidSequencePlanner::sindex--;

                IntIter = CollisionAvoidSequencePlanner::BinarayPruneList.erase(IntIter);
                i--;
            }
            else
            {
                i++;
                IntIter++;
            }
        }
    }

    CollisionAvoidSequencePlanner::BinarayPruneList.push_back(currBinCut);
    sindex++;
    return false;
}



bool CollisionAvoidSequencePlanner::Debug_Is_Corrected()
{
    return true;
}

bool CollisionAvoidSequencePlanner::Debug_Do_Something()
{
    return true;
}
