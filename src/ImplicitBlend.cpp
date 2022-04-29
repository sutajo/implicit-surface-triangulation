/*
 * ImplicitBlend
 *
 * File: 	ImplicitBlend.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 6 2015
 */

#include "ImplicitBlend.hpp"

using namespace Implicit;

Blend::Blend(Object* left, Object* right) :
	Operator(left, right)
{
	compute_bounds();
}

Blend::Blend(Object* left, Object* right, double iso) :
	Operator(left, right, iso)
{
	compute_bounds();
}

double Blend::Evaluate(const glm::dvec3& point)
{
	return FieldValue(point) - m_iso;
}

double Blend::FieldValue(const glm::dvec3& point)
{
	if (!m_bounds.contains(point)) return 0;

#ifdef DEBUG
	const float left_value = m_left_child->FieldValue(point);
	const float right_value = m_right_child->FieldValue(point);
	std::cout << "[" << left_value << "," << right_value << "]\t";
#endif
	return m_left_child->FieldValue(point) + m_right_child->FieldValue(point);

}

glm::dvec3 Blend::Normal(const glm::dvec3& point)
{
	const double right_contrib = m_right_child->FieldValue(point);
	const double left_contrib = m_left_child->FieldValue(point);
	return  glm::normalize((right_contrib * m_right_child->Normal(point) +
				left_contrib * m_left_child->Normal(point))
			/ (left_contrib + right_contrib));
}

void Blend::compute_bounds()
{
	m_bounds.add(m_left_child->GetBoundingBox());
	m_bounds.add(m_right_child->GetBoundingBox());
}


