#include "glmMeshAdaptor.hpp"
#include <glm/gtx/norm.hpp>

namespace glm {

	double norm(const glm::dvec3& x) {
		return glm::length(x);
	}

	double sqrnorm(const glm::dvec3& x)
	{
		return glm::length2(x);
	}

	glm::dvec3& vectorize(glm::dvec3& x, double const& val)
	{
		x.x = val;
		x.y = val;
		x.z = val;
		return x;
	}
}

OpenMesh::SmartHalfedgeHandle FindBoundaryHalfEdge(const GlmTriMesh& mesh)
{
	OpenMesh::SmartHalfedgeHandle heh;
	for (auto& edge : mesh.edges())
	{
		if (edge.is_boundary())
		{
			if (edge.h0().is_boundary())
				heh = edge.h0();
			else
				heh = edge.h1();

			break;
		}
	}
	return heh;
}