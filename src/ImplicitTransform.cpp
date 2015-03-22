/*
 * ImplicitTransform
 *
 * File: 	ImplicitTransform.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#include "ImplicitTransform.hpp"

using namespace Implicit;

Transform::Transform();
	Object(),
	m_to_local(),
	m_from_local()
{
}

Transform::Transform(Object* obj, const glm::mat4& m) :
	Object(),
	m_right_object(NULL),
	m_left_object(obj),
	m_to_local(m),
	m_from_local()
{
	setWorldMatrix(m);
}

float Transform::Evaluate(glm::vec3 point)
{
	return m_left_object->Evaluate(map_to(point));
}

float Transform::FieldValue(glm::vec3 point)
{
	return m_left_object->FieldValue(map_to(point));
}

glm::vec3 Transform::map_to(glm::vec3 v)
{
	return glm::vec3(glm::vec4(v, 1.f) * m_to_local);
}

glm::vec3 Transform::map_from(glm::vec3 v)
{
	return glm::vec3(glm::vec4(v, 1.f) * m_from_local);
}
