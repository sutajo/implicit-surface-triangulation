/*
 * floatlibs
 *
 * File: 	floatlibs.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Jan 31 2015
 */

#ifndef FLOATLIB_H
#define FLOATLIB_H

#include <cfloat>

#include <iostream>


inline double f_equ(double a, double b)
{
	return (a <= b + DBL_EPSILON && a >= b - DBL_EPSILON);
}

inline double f_equ(double a, double b, double eps)
{
	return (a <= b + eps && a >= b - eps);
}

inline bool f_is_zero(double a)
{
	return f_equ(a, 0);
}

inline int f_sign(double x)
{
	return (f_is_zero(x) ? 0 : ( x < 0 ? -1 : 1 ));
}

inline float f_ge(double a, double b)
{
	//std::cerr << a << ">=" << b << '\n';
	return a >= b - DBL_EPSILON;
}

inline double f_le(double a, double b)
{
	//std::cerr << a << "<=" << b << '\n';
	return a <= b + DBL_EPSILON;
}

inline double f_lt(double a, double b)
{
	//std::cerr << a << '<' << b << '\n';
	return (a < b + DBL_EPSILON || a < b - DBL_EPSILON);
}

inline double f_gt(double a, double b)
{
	//std::cerr << a << '>' << b << '\n';
	return (a > b - DBL_EPSILON || a > b + DBL_EPSILON);
}

inline double f_div(double a, double b)
{
	if (f_is_zero(b))
	{
		if (f_is_zero(a)) return 0;
		else return DBL_MAX * f_sign(a);
	}
	else
	{
		if (f_is_zero(a)) return 0;
		else
		{
			if ((a + b) == a)
				return DBL_MAX * f_sign(a) * f_sign(b);
			else return a / b;
		}
	}
}


#endif //FLOATLIB_H
