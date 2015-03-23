/*
 * main
 *
 * File: 	main.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#include "ImplicitSystem.hpp"
#include <iostream>
#include "vecHelp.hpp"

int main()
{
	Implicit::Primitive p(geoffFunction);
	std::cout << p.GetStartVertex() << '\n';
	return 0;
}
