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
{ }

Union::Union(Object* left, Object* right, float iso) :
	Operator(left, right, iso)
{ }

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
	float left_field_value = m_left_child->FieldValue(point);
	float right_field_value = m_right_child->FieldValue(point);
	float total_field_value = left_field_value + right_field_value;

	float left_contrib = left_field_value / total_field_value;
	float right_contrib = right_field_value / total_field_value;

	return glm::normalize(m_left_child->Normal(point) * left_contrib +
		m_right_child->Normal(point) * right_contrib);
}
