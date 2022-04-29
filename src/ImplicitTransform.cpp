/*
 * ImplicitTransform
 *
 * File: 	ImplicitTransform.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#include "ImplicitTransform.hpp"

using namespace Implicit;

Transform::Transform(Object* obj, const glm::dmat4& m) :
	Object(),
	m_child(obj),
	m_to_local(m),
	m_from_local()
{
	setWorldMatrix(m);
}

Transform::Transform(Object* obj) :
	m_child(obj)
{
}

double Transform::Evaluate(const glm::dvec3& point)
{
	return m_child->Evaluate(map_to(point));
}

double Transform::FieldValue(const glm::dvec3& point)
{
	if (m_bounds.contains(point))
		return m_child->FieldValue(map_to(point));
	else return 0;
}

glm::dvec3 Transform::GetStartVertex()
{
	return (map_from(m_child->GetStartVertex()));
}

glm::dvec3 Transform::GetCenterVertex()
{
	return (map_from(m_child->GetCenterVertex()));
}

void Transform::setWorldMatrix(const glm::dmat4& m)
{
	m_bounds = m_child->GetBoundingBox();
	m_bounds.transform(m);
	m_from_local = m;
	m_to_local = glm::inverse(m);
}

glm::dvec3 Transform::map_to(glm::dvec3 v)
{
	return glm::dvec3(m_to_local * glm::dvec4(v, 1.f));
}

glm::dvec3 Transform::map_from(glm::dvec3 v)
{
	return glm::dvec3(m_from_local * glm::dvec4(v, 1.f));
}
