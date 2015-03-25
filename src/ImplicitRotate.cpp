#include "ImplicitRotate.hpp"

using namespace Implicit;
using std::cos;
using std::sin;


Rotate::Rotate(Object* child, const glm::vec3& rot) :
	Transform(child)
{
	const float x = rot.x;
	const float y = rot.y;
	const float z = rot.z;
	glm::mat4 rotation;
	rotation[0][0] = cos(x) * cos(y);
	rotation[1][0] = cos(x) * sin(y) * sin(z) - sin(x) * cos(z);
	rotation[2][0] = cos(x) * sin(y) * cos(z) + sin(x) * cos(z);
	rotation[3][0] = 0;
	rotation[0][1] = sin(x) * cos(y);
	rotation[1][1] = sin(x) * sin(y) * sin(z) + cos(x) * cos(z);
	rotation[2][1] = sin(x) * sin(y) * cos(z) - cos(x) * sin(z);
	rotation[3][1] = 0;
	rotation[0][2] = -sin(y);
	rotation[1][2] = cos(y) * sin(z);
	rotation[2][2] = cos(y) * cos(z);
	rotation[3][2] = 0;
	rotation[0][3] = rotation[1][3] = rotation[2][3] = 0;
	rotation[3][3] = 1;

	setWorldMatrix(rotation);
}


Rotate::Rotate(Object* child, float x, float y, float z) :
	Transform(child)
{
	glm::mat4 rotation;
	rotation[0][0] = cos(x) * cos(y);
	rotation[1][0] = cos(x) * sin(y) * sin(z) - sin(x) * cos(z);
	rotation[2][0] = cos(x) * sin(y) * cos(z) + sin(x) * cos(z);
	rotation[3][0] = 0;
	rotation[0][1] = sin(x) * cos(y);
	rotation[1][1] = sin(x) * sin(y) * sin(z) + cos(x) * cos(z);
	rotation[2][1] = sin(x) * sin(y) * cos(z) - cos(x) * sin(z);
	rotation[3][1] = 0;
	rotation[0][2] = -sin(y);
	rotation[1][2] = cos(y) * sin(z);
	rotation[2][2] = cos(y) * cos(z);
	rotation[3][2] = 0;
	rotation[0][3] = rotation[1][3] = rotation[2][3] = 0;
	rotation[3][3] = 1;

	setWorldMatrix(rotation);
}

Rotate::Rotate(Object* child, const glm::vec3& axis, float angle) :
	Transform(child)
{
	setWorldMatrix(glm::rotate(glm::mat4(), angle, axis));
}
