#include "ImplicitLine.hpp"

using namespace Implicit;

Line::Line(FieldFunction f) :
	Primitive(f),
	m_length(1.),
	m_endpoint_1(m_length/-2., 0, 0),
	m_endpoint_2(m_length/2., 0, 0)
{
	compute_bounds();
}

Line::Line(FieldFunction f, double iso) :
	Primitive(f, iso),
	m_length(1.),
	m_endpoint_1(m_length/-2., 0, 0),
	m_endpoint_2(m_length/2., 0, 0)
{
	compute_bounds();
}

Line::Line(FieldFunction f, double iso, double radius) :
	Primitive(f, iso, radius),
	m_length(1.),
	m_endpoint_1(m_length/-2., 0, 0),
	m_endpoint_2(m_length/2., 0, 0)
{
	compute_bounds();
}

Line::Line(FieldFunction f, double iso, double radius, double length) :
	Primitive(f, iso, radius),
	m_length(length),
	m_endpoint_1(m_length/-2., 0, 0),
	m_endpoint_2(m_length/2., 0, 0)
{ }

double Line::Evaluate(const glm::dvec3& point)
{
	return Primitive::Evaluate(getDistance(point));
}

double Line::FieldValue(const glm::dvec3& point)
{
	return Primitive::FieldValue(getDistance(point));
}

glm::dvec3 Line::GetStartVertex()
{
	return project(glm::dvec3(0, 0, 0));
}

glm::dvec3 Line::GetCenterVertex()
{
	return glm::dvec3(0, 0, 0);
}

glm::dvec3 Line::Normal(const glm::dvec3& point)
{
	return glm::normalize(point - getNearest(point));
}

void Line::compute_bounds()
{
	std::list<glm::dvec3> points;
	points.push_back(m_endpoint_1 - glm::dvec3(m_radius, m_radius, m_radius));
	points.push_back(m_endpoint_2 + glm::dvec3(m_radius, m_radius, m_radius));
	m_bounds.compute(points);
}

glm::dvec3 Line::getNearest(const glm::dvec3& point)
{
	double proj = glm::dot(point - m_endpoint_1,
			m_endpoint_2 - m_endpoint_1) / (m_length * m_length);
	if (proj < 0.) return m_endpoint_1;
	if (proj > 1.) return m_endpoint_2;
	return m_endpoint_1 + proj * (m_endpoint_2 - m_endpoint_1);
}

double Line::getDistance(const glm::dvec3& point)
{
	// If the line is short enough, pretend it is a point
	if (m_length * m_length <= 0.025)
		return glm::length(point - m_endpoint_1);
	return glm::length(point - getNearest(point));
}

