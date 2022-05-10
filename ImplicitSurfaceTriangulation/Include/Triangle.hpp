#pragma once

#include "glmMeshAdaptor.hpp"

#include <glm/glm.hpp>
#include <tuple>
#include <string>
#include <array>

struct Triangle
{
	glm::dvec3 a;
	glm::dvec3 b;
	glm::dvec3 c;

	Triangle(){}
	Triangle(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) : a(a), b(b), c(c) {}

	template<typename mesh_t>
	Triangle(const OpenMesh::SmartFaceHandle face, const mesh_t& mesh)
	{
		if (face.valence() != 3)
			throw std::runtime_error("Face is not a triangle");

		auto vertices = face.vertices().to_array<3>();
		a = mesh.point(vertices[0]), b = mesh.point(vertices[1]), c = mesh.point(vertices[2]);
	}

	std::tuple<double, double, double> GetSideLengths() const;
	double GetLongestSide() const;
	glm::dvec3 GetNormal() const;
	glm::dvec3 GetAltitude(int vertex) const;
	glm::dvec3 GetCentroid() const;
	glm::dvec3 ProjectPoint(const glm::dvec3& point) const;
	double GetDistanceFrom(const glm::dvec3& point) const;

	std::string to_string() const; 
};

static_assert(sizeof(Triangle) == 3 * sizeof(glm::dvec3));