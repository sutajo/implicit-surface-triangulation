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
	Implicit::Translate pt(&p, 1, 0, 10);
	std::cout << "P: " << '\n';
	std::cout << "Start: " << p.GetStartVertex() << '\n';
	std::cout << "Normal: " << p.Normal(p.GetStartVertex()) << '\n';
	std::cout << "Center: " << p.GetCenterVertex() << '\n';
	std::cout << "Start: " << pt.GetStartVertex() << '\n';
	std::cout << "Normal: " << pt.Normal(pt.GetStartVertex()) << '\n';
	std::cout << "Center: " << pt.GetCenterVertex() << '\n';

	return 0;
}
