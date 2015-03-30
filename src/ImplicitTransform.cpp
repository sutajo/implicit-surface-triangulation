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

Transform::Transform(Object* obj) :
	m_child(obj)
{ }

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
	return glm::normalize(
			glm::vec3(m_normal_conversion *
				glm::vec4(m_child->Normal(map_to(point)), 1.f)
				)
			);
}

glm::vec3 Transform::GetStartVertex()
{
	return (map_from(m_child->GetStartVertex()));
}

glm::vec3 Transform::GetCenterVertex()
{
	return (map_from(m_child->GetCenterVertex()));
}

Aabb Transform::GetBoundingBox()
{
	Aabb ret_box;
	const Aabb child_aabb = m_child->GetBoundingBox();
	ret_box.include(map_from(child_aabb.min()));
	ret_box.include(map_from(child_aabb.max()));
	return ret_box;
}

void Transform::setWorldMatrix(const glm::mat4& m)
{
	m_from_local = m;
	m_to_local = glm::inverse(m);
	m_normal_conversion = glm::inverseTranspose(m);
}

glm::vec3 Transform::map_to(glm::vec3 v)
{
	return glm::vec3(m_to_local * glm::vec4(v, 1.f));
}

glm::vec3 Transform::map_from(glm::vec3 v)
{
	return glm::vec3(m_from_local * glm::vec4(v, 1.f));
}
