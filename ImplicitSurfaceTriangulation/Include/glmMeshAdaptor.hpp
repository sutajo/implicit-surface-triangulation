#pragma once

#include <glm/glm.hpp>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/vector_traits.hh>

// OpenMesh vertex traits for glm::vec3
namespace OpenMesh {
	template<>
	struct OpenMesh::vector_traits<glm::dvec3>
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
	glm::dvec3& vectorize(glm::dvec3& x, double const& val);
}

// OpenMesh vertex
struct GlmVertex : public OpenMesh::DefaultTraits
{
	using Point = glm::dvec3;
	using Normal = glm::dvec3;
	EdgeTraits
	{
		bool grownAlready = false;
	};
	FaceTraits
	{
		bool createdByEarCutting = false;
	};
};

// Concrete mesh type
using GlmMesh = OpenMesh::TriMesh_ArrayKernelT<GlmVertex>;