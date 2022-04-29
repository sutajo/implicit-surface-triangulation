/*
 * ImplicitRotate
 *
 * File: 	ImplicitRotate.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 25 2015
 */

#include "ImplicitRotate.hpp"

using namespace Implicit;

Rotate::Rotate(Object* child, const glm::dvec3& axis, double angle) :
	Transform(child)
{
	setWorldMatrix(glm::rotate(glm::dmat4(1.0), angle, glm::normalize(axis)));
}

glm::dvec3 Rotate::Normal(const glm::dvec3& p)
{
	return map_from(m_child->Normal(map_to(p)));
}
