/*
 * ImplicitPrimitive
 *
 * File: 	ImplicitPrimitive.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Feb 26 2015
 */

#include "ImplicitPrimitive.hpp"

using namespace Implicit;

Primitive::Primitive(FieldFunction f) :
	Object(),
	m_fieldFunction(f),
	m_radius(1)
{
}

Primitive::Primitive(FieldFunction f, double iso) :
	Object(iso),
	m_fieldFunction(f),
	m_radius(1)
{
}

Primitive::Primitive(FieldFunction f, double iso, double radius) :
	Object(iso),
	m_fieldFunction(f),
	m_radius(radius)
{
}


double Primitive::FieldValue(double r)
{
	return m_fieldFunction(r, m_radius);
}

double Primitive::Evaluate(double r)
{
	return FieldValue(r) - m_iso;
}

