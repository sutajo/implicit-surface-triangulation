#include "ImplicitTorus.hpp"

using namespace Implicit;

Torus::Torus() :
	Differentiated(2.0),
	m_cross_radius(0.5),
	m_center_radius(3.0)
{
	SetEquation();
	compute_bounds();
}

Torus::Torus(double iso) :
	Differentiated(iso),
	m_cross_radius(0.5),
	m_center_radius(3.0)
{
	SetEquation();
	compute_bounds();
}

Torus::Torus(double iso, double i_r, double o_r) :
	Differentiated(iso),
	m_cross_radius(i_r),
	m_center_radius(o_r)
{
	SetEquation();
	compute_bounds();
}

double Torus::Evaluate(const glm::dvec3& point)
{
	return FieldValue(point)-GetIso();
}

double Torus::FieldValue(const glm::dvec3& point)
{
	return Differentiated::FieldValue(point);
}

glm::dvec3 Torus::GetStartVertex()
{
	return getNearest(GetCenterVertex());
}

glm::dvec3 Torus::GetCenterVertex()
{
	return glm::dvec3(0.0f);
}

glm::dvec3 Torus::Normal(const glm::dvec3& point)
{
	return Differentiated::Normal(point);
}

double Torus::getDistance(const glm::dvec3& point)
{
	return glm::length(point - getNearest(point));
}

void Torus::compute_bounds()
{
	std::list<glm::dvec3> points;
	points.push_back(glm::dvec3(m_center_radius + m_cross_radius, 0, 0));
	points.push_back(glm::dvec3(-(m_center_radius + m_cross_radius), 0, 0));
	points.push_back(glm::dvec3(0, m_center_radius + m_cross_radius, 0));
	points.push_back(glm::dvec3(0, -(m_center_radius + m_cross_radius), 0));
	points.push_back(glm::dvec3(0, 0, m_cross_radius));
	points.push_back(glm::dvec3(0, 0, -m_cross_radius));
	m_bounds.compute(points);
}

void Implicit::Torus::SetEquation()
{
	Equation = [&](const autodiff::dual& x, const autodiff::dual& y, const autodiff::dual& z)
	{
		auto diff = sqrt(x * x + y * y) - m_center_radius;
		return diff*diff + z*z - (m_cross_radius*m_cross_radius);
	};
}

glm::dvec3 Torus::getNearest(const glm::dvec3& pt)
{
	const auto projected = glm::dvec3(pt.x, pt.y, 0);
	glm::dvec3 torusInnerPoint;
	if (glm::length(projected) > DBL_MIN)
		torusInnerPoint = glm::normalize(projected) * m_center_radius;
	else
		torusInnerPoint = glm::dvec3(-m_center_radius, 0.0, 0.0);

	return findRootBetween(torusInnerPoint, pt, 100);
}
