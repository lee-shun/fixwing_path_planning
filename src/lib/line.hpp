#ifndef _LINE_HPP_
#define _LINE_HPP_
/*************************************************************************************
*
*Author:
*
*Date:
*
*Description:
*
*
*
*************************************************************************************/
#include "vector.hpp"
#include "mathlib.hpp"

struct Line {
	Point A, B;
	double l;
	bool operator<(const Line& b)const {
		if (dcmp(A.x - b.A.x) == 0)return A.y<b.A.y;
		return A.x<b.A.x;
	}
};

#endif
