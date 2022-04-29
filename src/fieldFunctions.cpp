/*
 * fieldFunctions
 *
 * File: 	fieldFunctions.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Feb 19 2015
 */

#include "fieldFunctions.hpp"

double linearFunction(double r, double R)
{
	return R - r;
}

double geoffFunction(double r, double R)
{
	if (r >= R) return 0;
	const double rs = (r * r) / (R * R);
	const double rq = rs * rs;
	const double rse = rq * rs;
	return 1 - (4.f/9.f) * rse + (17.f/9.f) * rq - (22.f/9.f) * rs;
}

double metaballFunction(double r, double R)
{
	if (r <= (R / 3.f))
		return 1 - (3 * (r * r ) / (R * R));
	else if (r < R)
		return 1.5 * (1 - r) * (1 - r);
	else return 0;
}
