#include "ImplicitSphere.hpp"

using namespace Implicit;

Sphere::Sphere(FieldFunction f) :
	Primitive(f)
{
	compute_bounds();
}

Sphere::Sphere(FieldFunction f, float iso) :
	Primitive(f, iso)
{
	compute_bounds();
}

Sphere::Sphere(FieldFunction f, float iso, float radius) :
	Primitive(f, iso, radius)
{
	compute_bounds();
}

float Sphere::Evaluate(const glm::vec3& p)
{
	return Primitive::Evaluate(getDistance(p));
}

float Sphere::FieldValue(const glm::vec3& p)
{
	return Primitive::FieldValue(getDistance(p));
}



glm::vec3 Sphere::GetStartVertex()
{
	return project(glm::vec3(0, 0, 0));
}

glm::vec3 Sphere::GetCenterVertex()
{
	return glm::vec3(0, 0, 0);
}

void Sphere::compute_bounds()
{
	std::list<glm::vec3> points;
	points.push_back(glm::vec3(m_radius, 0, 0) );
	points.push_back(glm::vec3(-m_radius, 0, 0));
	points.push_back(glm::vec3(0, m_radius, 0) );
	points.push_back(glm::vec3(0, -m_radius, 0));
	points.push_back(glm::vec3(0, 0, m_radius) );
	points.push_back(glm::vec3(0, 0, -m_radius));
	m_bounds.compute(points);
}


glm::vec3 Sphere::Normal(const glm::vec3& point)
{
	if (point == glm::vec3(0, 0, 0)) return glm::vec3(0, 1, 0);
	return glm::normalize(point);
}

float Sphere::getDistance(const glm::vec3& p)
{
	return glm::length(p);
}
