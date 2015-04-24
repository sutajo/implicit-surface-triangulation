/*
 * ImplicitTranslate
 *
 * File: 	ImplicitTranslate.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 23 2015
 */

#include "ImplicitTranslate.hpp"

using namespace Implicit;

Translate::Translate(Object* child, const glm::vec3& direction) :
	Transform(child)
{
	setWorldMatrix(glm::translate(glm::mat4(), direction));
}

Translate::Translate(Object *child, float x, float y, float z) :
	Transform(child)
{
	setWorldMatrix(glm::translate(glm::mat4(), glm::vec3(x, y, z)));
}

glm::vec3 Translate::Normal(const glm::vec3& p)
{
	return m_child->Normal(map_to(p));
}
