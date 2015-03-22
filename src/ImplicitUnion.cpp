/*
 * ImplicitUnion
 *
 * File: 	ImplicitUnion.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#include "ImplicitUnion.hpp"

using namespace Implicit;

Union::Union() :
	Object()
{ }

Union::Union(float iso) :
	Object(iso)
{ }

float Union::Evaluate(glm::vec3 point)
{
	return FieldValue(point) - m_iso;
}

float Union::FieldValue(glm::vec3 point)
{
	return std::max(m_left_object->FieldValue(point),
			m_right_object->FieldValue(point));
}

glm::vec3 Union::Normal(glm::vec3 point)
{
	float left_field_value = m_left_object->FieldValue(point);
	float right_field_value = m_right_object->FieldValue(point);
	float total_field_value = left_field_value + right_field_value;

	float left_contrib = left_field_value / total_field_value;
	float right_contrib = right_field_value / total_field_value;

	return glm::normalize(m_left_object->Normal(point) * left_contrib +
		m_right_object->Normal(point) * right_contrib);
}
