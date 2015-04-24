/*
 * fieldFunctions
 *
 * File: 	fieldFunctions.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Feb 19 2015
 */

#include "fieldFunctions.hpp"

float linearFunction(float r, float R)
{
	return ((r*r) >= (R*R)) ? 0 : 1 * R - r;
}

float geoffFunction(float r, float R)
{
	if (r >= R) return 0;
	register const float rs = (r * r) / (R * R);
	register const float rq = rs * rs;
	register const float rse = rq * rs;
	return 1 - (4.f/9.f) * rse + (17.f/9.f) * rq - (22.f/9.f) * rs;
}

float metaballFunction(float r, float R)
{
	if (r <= (R / 3.f))
		return 1 - (3 * (r * r ) / (R * R));
	else if (r < R)
		return 1.5 * (1 - r) * (1 - r);
	else return 0;
}
