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

Primitive::Primitive(FieldFunction f, float iso) :
	Object(iso),
	m_fieldFunction(f),
	m_radius(1)
{
}

Primitive::Primitive(FieldFunction f, float iso, float radius) :
	Object(iso),
	m_fieldFunction(f),
	m_radius(radius)
{
}


float Primitive::FieldValue(float r)
{
	return m_fieldFunction(r, m_radius);
}

float Primitive::Evaluate(float r)
{
	return FieldValue(r) - m_iso;
}

