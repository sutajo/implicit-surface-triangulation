#include "glmMeshAdaptor.hpp"


namespace glm {

	double norm(const glm::dvec3& x) {
		return glm::length(x);
	}

	glm::dvec3& vectorize(glm::dvec3& x, double const& val)
	{
		x.x = val;
		x.y = val;
		x.z = val;
		return x;
	}
}