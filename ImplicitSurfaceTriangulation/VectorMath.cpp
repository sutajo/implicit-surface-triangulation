#include "VectorMath.hpp"

glm::dvec3 Project(const glm::dvec3& projected, const glm::dvec3 &target)
{
	return glm::dot(target,projected) / glm::dot(target, target) * target ;
}
