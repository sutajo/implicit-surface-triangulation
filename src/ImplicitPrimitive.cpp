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
{ }

Primitive::Primitive(FieldFunction f, float iso) :
	Object(iso),
	m_fieldFunction(f),
	m_radius(1)
{ }

Primitive::Primitive(FieldFunction f, float iso, float radius) :
	Object(iso),
	m_fieldFunction(f),
	m_radius(radius)
{ }


float Primitive::FieldValue(float r)
{
	return m_fieldFunction(r, m_radius);
}

float Primitive::Evaluate(float r)
{
	return FieldValue(r) - m_iso;
}

float Primitive::Evaluate(const glm::vec3& p)
{
	return Evaluate(getDistance(p));
}

float Primitive::FieldValue(const glm::vec3& p)
{
	return FieldValue(getDistance(p));
}

glm::vec3 Primitive::Normal(const glm::vec3& point)
{
	if (point == glm::vec3(0, 0, 0)) return glm::vec3(0, 1, 0);
	return glm::normalize(point);
}

float Primitive::getDistance(const glm::vec3& p)
{
	return glm::length(p);
}

glm::vec3 Primitive::GetStartVertex()
{
	return Project(glm::vec3(0, 0, 0));
}


