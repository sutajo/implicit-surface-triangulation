#pragma once

#include <glm/glm.hpp>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/vector_traits.hh>

// OpenMesh vertex traits for glm::vec3
namespace OpenMesh {
	template<>
	struct vector_traits<glm::dvec3>
	{
		using vector_type = glm::dvec3;
		using value_type = glm::dvec3::value_type;
		static const size_t size_ = glm::dvec3::length();
		static size_t size() { return glm::dvec3::length(); }
	};

}

// Functions for glm::vec3 that OpenMesh uses
namespace glm {
	double norm(const glm::dvec3& x);
	double sqrnorm(const glm::dvec3& x);
	glm::dvec3& vectorize(glm::dvec3& x, double const& val);
}

enum class FaceCreationMethod
{
	Seed,
	IsoscelesGrowing,
	EarCutting,
	SmallPolygonFilling,
	SubdivisionOnBridges,
	XFilling,
	EarFilling,
	ConvexPolygonFilling,
	RelaxedEarFilling,
	ConcaveVertexBisection
};


// OpenMesh vertex
struct GlmAdaptor : public OpenMesh::DefaultTraits
{
	using Point = glm::dvec3;
	using Normal = glm::dvec3;
	VertexTraits
	{
		OpenMesh::VertexHandle closestNeighbour;
		OpenMesh::VertexHandle connectedVertex;
	};
	EdgeTraits
	{
		bool grownAlready = false;
	};
	FaceTraits
	{
		FaceCreationMethod faceCreationMethod;
	};
};

// Concrete mesh type
using GlmTriMesh = OpenMesh::TriMesh_ArrayKernelT<GlmAdaptor>;
using GlmPolyMesh = OpenMesh::PolyMesh_ArrayKernelT<GlmAdaptor>;

template<typename mesh_t>
OpenMesh::SmartHalfedgeHandle FindBoundaryHalfEdge(const mesh_t& mesh)
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