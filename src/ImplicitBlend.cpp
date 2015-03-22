/*
 * ImplicitBlend
 *
 * File: 	ImplicitBlend.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 6 2015
 */

#include "ImplicitBlend.hpp"

using namespace Implicit;

Blend::Blend() :
	Object()
{ }

Blend::Blend(float iso) :
	Object(iso)
{ }

float Blend::Evaluate(glm::vec3 point)
{
	return FieldValue(point) - m_iso;
}

float Blend::FieldValue(glm::vec3 point)
{
	float left_fvalue = 0;
	float right_fvalue = 0;
	if (m_left_object) left_fvalue = m_left_object->FieldValue(point);
	if (m_right_object) right_fvalue = m_right_object->FieldValue(point);
	return left_fvalue + right_fvalue;
}

glm::vec3 Blend::Normal(glm::vec3 point)
{

	float left_field_value;
	float right_field_value;
	float total_field_value;
	glm::vec3 left_normal;
	glm::vec3 right_normal;

	if (m_left_object)
	{
		left_field_value = m_left_object->FieldValue(point);
		left_normal = m_left_object->Normal(point);
	}
	if (m_right_object)
	{
		right_field_value->m_right_object->FieldValue(point);
		right_normal = m_right_object->Normal(point);
	}
	total_field_value = left_field_value + right_field_value;

	float left_contrib = left_field_value / total_field_value;
	float right_contrib = right_field_value / total_field_value;

	return glm::normalize(right_normal * right_contrib +
			left_normal * left_contrib);

}
