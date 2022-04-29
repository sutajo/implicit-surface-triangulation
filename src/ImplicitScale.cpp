/*
 * ImplicitScale
 *
 * File: 	ImplicitScale.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 25 2015
 */

#include "ImplicitScale.hpp"
#include <glm/gtx/transform.hpp>

using namespace Implicit;

Scale::Scale(Object* child, const glm::vec3& scale) :
	Transform(child)
{
	glm::mat4 transform;
	transform[0][0] = scale.x;
	transform[1][1] = scale.y;
	transform[2][2] = scale.z;
	m_normal_conversion = glm::inverseTranspose(transform);
	setWorldMatrix(transform);
	auto bb = child->GetBoundingBox();
	bb.transform(glm::scale(scale));
	m_bounds = bb;
}

Scale::Scale(Object* child, float x, float y, float z) :
	Transform(child)
{
	glm::mat4 transform;
	transform[0][0] = x;
	transform[1][1] = y;
	transform[2][2] = z;
	m_normal_conversion = glm::inverseTranspose(transform);
	setWorldMatrix(transform);
}

Scale::Scale(Object* child, float s) :
	Transform(child)
{
	m_normal_conversion = glm::inverseTranspose(glm::mat4(s));
	setWorldMatrix(glm::mat4(s));
}

glm::vec3 Scale::Normal(const glm::vec3& p)
{
	return glm::normalize(
			glm::vec3(m_normal_conversion *
				glm::vec4(m_child->Normal(map_to(p)), 1.f)));
}
