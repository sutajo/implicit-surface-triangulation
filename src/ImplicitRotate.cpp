/*
 * ImplicitRotate
 *
 * File: 	ImplicitRotate.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 25 2015
 */

#include "ImplicitRotate.hpp"

using namespace Implicit;

Rotate::Rotate(Object* child, const glm::vec3& axis, float angle) :
	Transform(child)
{
	setWorldMatrix(glm::rotate(glm::mat4(), angle, glm::normalize(axis)));
}

glm::vec3 Rotate::Normal(const glm::vec3& p)
{
	return map_from(m_child->Normal(map_to(p)));
}
