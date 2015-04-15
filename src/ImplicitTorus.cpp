#include "ImplicitTorus.hpp"

using namespace Implicit;

Torus::Torus(FieldFunction f) :
	Primitive(f),
	m_inner_radius(0.5),
	m_outer_radius(0.6),
	m_cross_radius(m_outer_radius - m_inner_radius),
	m_center_radius(m_inner_radius + m_cross_radius / 2.f)
{
	compute_bounds();
}

Torus::Torus(FieldFunction f, float iso) :
	Primitive(f, iso),
	m_inner_radius(0.5),
	m_outer_radius(0.6),
	m_cross_radius(m_outer_radius - m_inner_radius),
	m_center_radius(m_inner_radius + m_cross_radius / 2.f)
{
	compute_bounds();
}

Torus::Torus(FieldFunction f, float iso, float i_r, float o_r) :
	Primitive(f, iso, o_r - i_r),
	m_inner_radius(i_r),
	m_outer_radius(o_r),
	m_cross_radius(o_r - i_r ),
	m_center_radius(m_inner_radius + m_cross_radius / 2.f)
{
	compute_bounds();
}

float Torus::Evaluate(const glm::vec3& point)
{
	return Primitive::Evaluate(getDistance(point));
}

float Torus::FieldValue(const glm::vec3& point)
{
	return Primitive::FieldValue(getDistance(point));
}

glm::vec3 Torus::GetStartVertex()
{
	return project(glm::vec3(m_center_radius, 0, 0));
}

glm::vec3 Torus::GetCenterVertex()
{
	return glm::vec3(m_center_radius, 0, 0);
}

glm::vec3 Torus::Normal(const glm::vec3& point)
{
	return glm::normalize(point - getNearest(point));
}


float Torus::getDistance(const glm::vec3& point)
{
	return glm::length(point - getNearest(point)) / m_cross_radius * 0.25;;
}

void Torus::compute_bounds()
{
	std::list<glm::vec3> points;
	points.push_back(glm::vec3(m_center_radius + m_cross_radius+0.21, 0, 0));
	points.push_back(glm::vec3(-(m_center_radius + m_cross_radius+0.21), 0, 0));
	points.push_back(glm::vec3(0, m_center_radius + m_cross_radius + 0.21, 0));
	points.push_back(glm::vec3(0, -(m_center_radius + m_cross_radius +0.21), 0));
	points.push_back(glm::vec3(0, 0, m_cross_radius));
	points.push_back(glm::vec3(0, 0, -m_cross_radius));
	m_bounds.compute(points);
}

glm::vec3 Torus::getNearest(const glm::vec3& pt)
{
	return glm::normalize(glm::vec3(pt.x, pt.y, 0)) * m_center_radius;
}
