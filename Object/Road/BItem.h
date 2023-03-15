#ifndef _BItem_
#define _BItem_

#pragma once
#include<string>
#include<list>
#include "BPointCoordinate.h"

using namespace std;
class BItem
{
public:
	BItem();
public: 
	int Id;
public: 
	string Name;
	
public: 
	list<BPointCoordinate> Contour;

public: 
	string GetName(void);
	  
};

#endif