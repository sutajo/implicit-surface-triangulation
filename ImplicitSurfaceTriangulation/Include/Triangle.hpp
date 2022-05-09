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
	Triangle(const OpenMesh::SmartFaceHandle face, const GlmTriMesh& mesh);

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