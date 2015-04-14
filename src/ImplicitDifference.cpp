/*
 * ImplicitDifference
 *
 * File: 	ImplicitDifference.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Apr 14 2015
 */

#include "ImplicitDifference.hpp"

using namespace Implicit;

Difference::Difference(Object* left, Object* right) :
	Operator(left, right)
{
	compute_bounds();
}

Difference::Difference(Object* left, Object* right, float iso) :
	Operator(left, right, iso)
{
	compute_bounds();
}

float Difference::Evaluate(const glm::vec3& point)
{
	return FieldValue(point) - m_iso;
}

float Difference::FieldValue(const glm::vec3& point)
{
	if (!m_bounds.contains(point)) return 0;
	return std::min(m_left_child->FieldValue(point),
			m_left_child->FieldValue(point) -
			m_right_child->FieldValue(point));
}

glm::vec3 Difference::Normal(const glm::vec3& point)
{
	if (m_left_child->FieldValue(point) <
			m_left_child->FieldValue(point) -
			m_right_child->FieldValue(point))
		return -m_right_child->Normal(point);
	return m_left_child->Normal(point);

}

void Difference::compute_bounds()
{
	m_bounds.add(m_left_child->GetBoundingBox());
}


