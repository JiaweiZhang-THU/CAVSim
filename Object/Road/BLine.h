#ifndef _BLine_
#define _BLine_

#pragma once
#include "BNode.h"
#include<cmath>

class BLine: public BItem
{
public: 
	BNode From_Node;

	BNode To_Node;
public: 
	string GetName(void);

	double GetLength(void);
	
	double GetArc(void);
	  
public: 
	bool Contains(BNode point);

	double GetContains(BNode point);

public: 
	BLine(void);
};

#endif // !_BLine_