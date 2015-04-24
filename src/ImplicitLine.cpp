#include "ImplicitLine.hpp"

using namespace Implicit;

Line::Line(FieldFunction f) :
	Primitive(f),
	m_length(1.f),
	m_endpoint_1(m_length/-2.f, 0, 0),
	m_endpoint_2(m_length/2.f, 0, 0)
{
	compute_bounds();
}

Line::Line(FieldFunction f, float iso) :
	Primitive(f, iso),
	m_length(1.f),
	m_endpoint_1(m_length/-2.f, 0, 0),
	m_endpoint_2(m_length/2.f, 0, 0)
{
	compute_bounds();
}

Line::Line(FieldFunction f, float iso, float radius) :
	Primitive(f, iso, radius),
	m_length(1.f),
	m_endpoint_1(m_length/-2.f, 0, 0),
	m_endpoint_2(m_length/2.f, 0, 0)
{
	compute_bounds();
}

Line::Line(FieldFunction f, float iso, float radius, float length) :
	Primitive(f, iso, radius),
	m_length(length),
	m_endpoint_1(m_length/-2.f, 0, 0),
	m_endpoint_2(m_length/2.f, 0, 0)
{ }

float Line::Evaluate(const glm::vec3& point)
{
	return Primitive::Evaluate(getDistance(point));
}

float Line::FieldValue(const glm::vec3& point)
{
	return Primitive::FieldValue(getDistance(point));
}

glm::vec3 Line::GetStartVertex()
{
	return project(glm::vec3(0, 0, 0));
}

glm::vec3 Line::GetCenterVertex()
{
	return glm::vec3(0, 0, 0);
}

glm::vec3 Line::Normal(const glm::vec3& point)
{
	return glm::normalize(point - getNearest(point));
}

void Line::compute_bounds()
{
	std::list<glm::vec3> points;
	points.push_back(m_endpoint_1 - glm::vec3(m_radius, m_radius, m_radius));
	points.push_back(m_endpoint_2 + glm::vec3(m_radius, m_radius, m_radius));
	m_bounds.compute(points);
}

glm::vec3 Line::getNearest(const glm::vec3& point)
{
	float proj = glm::dot(point - m_endpoint_1,
			m_endpoint_2 - m_endpoint_1) / (m_length * m_length);
	if (proj < 0.f) return m_endpoint_1;
	if (proj > 1.f) return m_endpoint_2;
	return m_endpoint_1 + proj * (m_endpoint_2 - m_endpoint_1);
}

float Line::getDistance(const glm::vec3& point)
{
	// If the line is short enough, pretend it is a point
	if (m_length * m_length <= 0.025f)
		return glm::length(point - m_endpoint_1);
	return glm::length(point - getNearest(point));
}

