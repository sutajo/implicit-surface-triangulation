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
	return m_left_child->FieldValue(point) + m_right_child->FieldValue(point);
}

glm::vec3 Blend::Normal(const glm::vec3& point)
{
	return glm::normalize(m_left_child->Normal(point) +
			m_right_child->Normal(point));
}

void Blend::compute_bounds()
{
	m_bounds.add(m_left_child->GetBoundingBox());
	m_bounds.add(m_right_child->GetBoundingBox());
}


