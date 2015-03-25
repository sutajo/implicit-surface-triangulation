/*
 * ImplicitIntersect
 *
 * File: 	ImplicitIntersect.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 25 2015
 */

#include "ImplicitIntersect.hpp"
using namespace Implicit;

Intersect::Intersect(Object* left, Object* right) :
	Operator(left, right)
{ }

Intersect::Intersect(Object* left, Object* right, float iso) :
	Operator(left, right, iso)
{ }

float Intersect::Evaluate(const glm::vec3& point)
{
	return FieldValue(point) - m_iso;
}

float Intersect::FieldValue(const glm::vec3& point)
{
	return std::min(m_left_child->FieldValue(point),
			m_right_child->FieldValue(point));
}

glm::vec3 Intersect::Normal(const glm::vec3& point)
{
	if (m_left_child->FieldValue(point) < m_right_child->FieldValue(point))
		return m_left_child->Normal(point);
	return m_right_child->Normal(point);
}


