/*
 * ImplicitBlend
 *
 * File: 	ImplicitBlend.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 6 2015
 */

#include "ImplicitBlend.hpp"

using namespace Implicit;

Blend::Blend(Object* left, Object* right) :
	Operator(left, right)
{
	compute_bounds();
}

Blend::Blend(Object* left, Object* right, float iso) :
	Operator(left, right, iso)
{
	compute_bounds();
}

float Blend::Evaluate(const glm::vec3& point)
{
	return FieldValue(point) - m_iso;
}

float Blend::FieldValue(const glm::vec3& point)
{
	if (!m_bounds.contains(point)) return 0;
	return m_left_child->FieldValue(point) + m_right_child->FieldValue(point);
}

glm::vec3 Blend::Normal(const glm::vec3& point)
{
	const float right_contrib = m_right_child->FieldValue(point);
	const float left_contrib = m_left_child->FieldValue(point);
	return  glm::normalize((right_contrib * m_right_child->Normal(point) +
				left_contrib * m_left_child->Normal(point))
			/ (left_contrib + right_contrib));
}

void Blend::compute_bounds()
{
	m_bounds.add(m_left_child->GetBoundingBox());
	m_bounds.add(m_right_child->GetBoundingBox());
}


