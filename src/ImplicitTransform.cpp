/*
 * ImplicitTransform
 *
 * File: 	ImplicitTransform.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#include "ImplicitTransform.hpp"

using namespace Implicit;

Transform::Transform(Object* obj, const glm::mat4& m) :
	Object(),
	m_child(obj),
	m_to_local(m),
	m_from_local()
{
	setWorldMatrix(m);
}

float Transform::Evaluate(const glm::vec3& point)
{
	return m_child->Evaluate(map_to(point));
}

float Transform::FieldValue(const glm::vec3& point)
{
	return m_child->FieldValue(map_to(point));
}

glm::vec3 Transform::Normal(const glm::vec3& point)
{
	return map_from(m_child->Normal(map_to(point)));
}

glm::vec3 Transform::GetStartVertex()
{
	return (map_from(m_child->GetStartVertex()));
}

glm::vec3 Transform::map_to(glm::vec3 v)
{
	return glm::vec3(glm::vec4(v, 1.f) * m_to_local);
}

glm::vec3 Transform::map_from(glm::vec3 v)
{
	return glm::vec3(glm::vec4(v, 1.f) * m_from_local);
}

void Transform::setWorldMatrix(const glm::mat4& m)
{
	m_from_local = glm::inverseTranspose(m);
	m_to_local = m;
}
