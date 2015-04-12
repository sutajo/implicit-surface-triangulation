/*
 * ImplicitIntersect
 *
 * File: 	ImplicitIntersect.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 25 2015
 */

#include "ImplicitIntersect.hpp"
using namespace Implicit;

Intersect::Intersect(Object* left, Object* right) :
	Operator(left, right)
{
	compute_bounds();
}

Intersect::Intersect(Object* left, Object* right, float iso) :
	Operator(left, right, iso)
{
	compute_bounds();
}

float Intersect::Evaluate(const glm::vec3& point)
{
	return FieldValue(point) - m_iso;
}

float Intersect::FieldValue(const glm::vec3& point)
{
	return std::min(m_left_child->FieldValue(point),
			m_right_child->FieldValue(point));
}

glm::vec3 Intersect::Normal(const glm::vec3& point)
{
	if (m_left_child->FieldValue(point) < m_right_child->FieldValue(point))
		return m_left_child->Normal(point);
	return m_right_child->Normal(point);
}

void Intersect::compute_bounds()
{
	glm::vec3 min;
	glm::vec3 max;
	const Aabb left_bb = m_left_child->GetBoundingBox();
	const Aabb right_bb = m_right_child->GetBoundingBox();
	min.x = left_bb.min().x > right_bb.min().x ? left_bb.min().x : right_bb.min().x;
	min.y = left_bb.min().y > right_bb.min().y ? left_bb.min().y : right_bb.min().y;
	min.z = left_bb.min().z > right_bb.min().z ? left_bb.min().z : right_bb.min().z;
	max.x = left_bb.max().x < right_bb.max().x ? left_bb.max().x : right_bb.max().x;
	max.y = left_bb.max().y < right_bb.max().y ? left_bb.max().y : right_bb.max().y;
	max.z = left_bb.max().z < right_bb.max().z ? left_bb.max().z : right_bb.max().z;
	m_bounds.include(min);
	m_bounds.include(max);
}
