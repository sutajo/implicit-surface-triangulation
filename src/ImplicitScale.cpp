#include "ImplicitScale.hpp"

using namespace Implicit;

Scale::Scale(Object* child, const glm::vec3& scale) :
	Transform(child)
{
	glm::mat4 transform;
	transform[0][0] = scale.x;
	transform[1][1] = scale.y;
	transform[2][2] = scale.z;
	setWorldMatrix(transform);
}

Scale::Scale(Object* child, float x, float y, float z) :
	Transform(child)
{
	glm::mat4 transform;
	transform[0][0] = x;
	transform[1][1] = y;
	transform[2][2] = z;
	setWorldMatrix(transform);
}

Scale::Scale(Object* child, float s) :
	Transform(child)
{
	setWorldMatrix(glm::mat4(s));
}
