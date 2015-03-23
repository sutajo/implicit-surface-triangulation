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
	glm::mat4 translation;
	translation[3][0] = direction.x;
	translation[3][1] = direction.y;
	translation[3][2] = direction.z;
	setWorldMatrix(translation);
}

Translate::Translate(Object *child, float x, float y, float z) :
	Transform(child)
{
	glm::mat4 translation;
	translation[3][0] = x;
	translation[3][1] = y;
	translation[3][2] = z;
	setWorldMatrix(translation);
}
