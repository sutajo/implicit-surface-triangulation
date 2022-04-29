/*
 * ImplicitTranslate
 *
 * File: 	ImplicitTranslate.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 23 2015
 */

#include "ImplicitTranslate.hpp"

using namespace Implicit;

Translate::Translate(Object* child, const glm::dvec3& direction) :
	Transform(child)
{
	auto w = glm::translate(glm::dmat4(1.0f), direction);
	setWorldMatrix(w);
	auto bb = child->GetBoundingBox();
	bb.transform(w);
	m_bounds = bb;
}

Translate::Translate(Object *child, double x, double y, double z) :
	Transform(child)
{
	setWorldMatrix(glm::translate(glm::dmat4(1.0f), glm::dvec3(x, y, z)));
	m_bounds = child->GetBoundingBox();
}

glm::dvec3 Translate::Normal(const glm::dvec3& p)
{
	return m_child->Normal(map_to(p));
}
