/*
 * ImplicitUnion
 *
 * File: 	ImplicitUnion.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#include "ImplicitUnion.hpp"

using namespace Implicit;

Union::Union(Object* left, Object* right) :
	Operator(left, right)
{
	compute_bounds();
}

Union::Union(Object* left, Object* right, float iso) :
	Operator(left, right, iso)
{
	compute_bounds();
}

float Union::Evaluate(const glm::vec3& point)
{
	return FieldValue(point) - m_iso;
}

float Union::FieldValue(const glm::vec3& point)
{
	return std::max(m_left_child->FieldValue(point),
			m_right_child->FieldValue(point));
}

glm::vec3 Union::Normal(const glm::vec3& point)
{
	if (m_left_child->FieldValue(point) > m_right_child->FieldValue(point))
		return m_left_child->Normal(point);
	return m_right_child->Normal(point);
}

void Union::compute_bounds()
{
	m_bounds.add(m_left_child->GetBoundingBox());
	m_bounds.add(m_right_child->GetBoundingBox());
}
