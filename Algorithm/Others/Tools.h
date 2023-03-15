#pragma once
#include <iostream>
#include <iomanip>
#include <sstream>

inline std::string doubleToStringRound2(double val)
{
    char* chCode;
    chCode = new char[20];
    sprintf(chCode, "%.2lf", val);
    std::string str(chCode);
    delete[]chCode;
    return str;
}

inline std::string doubleToStringRound0(double val)
{
    char* chCode;
    chCode = new char[20];
    sprintf(chCode, "%.0lf", val);
    std::string str(chCode);
    delete[]chCode;
    return str;
}
