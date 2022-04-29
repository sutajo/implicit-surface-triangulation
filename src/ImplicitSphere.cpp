#include "ImplicitSphere.hpp"

using namespace Implicit;

Sphere::Sphere(FieldFunction f) :
	Primitive(f)
{
	compute_bounds();
}

Sphere::Sphere(FieldFunction f, double iso) :
	Primitive(f, iso)
{
	compute_bounds();
}

Sphere::Sphere(FieldFunction f, double iso, double radius) :
	Primitive(f, iso, radius)
{
	compute_bounds();
}

double Sphere::Evaluate(const glm::dvec3& p)
{
	return Primitive::Evaluate(getDistance(p));
}

double Sphere::FieldValue(const glm::dvec3& p)
{
	return Primitive::FieldValue(getDistance(p));
}

glm::dvec3 Sphere::GetStartVertex()
{
	return project(glm::dvec3(0, 0, 0));
}

glm::dvec3 Sphere::GetCenterVertex()
{
	return glm::vec3(0, 0, 0);
}

void Sphere::compute_bounds()
{
	std::list<glm::dvec3> points;
	points.push_back(glm::vec3(m_radius, 0, 0) );
	points.push_back(glm::vec3(-m_radius, 0, 0));
	points.push_back(glm::vec3(0, m_radius, 0) );
	points.push_back(glm::vec3(0, -m_radius, 0));
	points.push_back(glm::vec3(0, 0, m_radius) );
	points.push_back(glm::vec3(0, 0, -m_radius));
	m_bounds.compute(points);
}


glm::dvec3 Sphere::Normal(const glm::dvec3& point)
{
	if (point == glm::dvec3(0, 0, 0)) return glm::dvec3(0, 1, 0);
	return glm::normalize(point);
}

double Sphere::getDistance(const glm::dvec3& p)
{
	return glm::length(p);
}
