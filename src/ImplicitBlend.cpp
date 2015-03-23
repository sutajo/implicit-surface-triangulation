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
{ }

Blend::Blend(Object* left, Object* right, float iso) :
	Operator(left, right, iso)
{ }

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
	float left_fv;
	float right_fv;
	float total_fv;

	left_fv = m_left_child->FieldValue(point);
	right_fv = m_right_child->FieldValue(point);
	const glm::vec3& left_normal = m_left_child->Normal(point);
	const glm::vec3& right_normal = m_right_child->Normal(point);
	total_fv = left_fv + right_fv;

	return glm::normalize(left_normal * (left_fv/total_fv) +
			right_normal * (right_fv/total_fv));
}
