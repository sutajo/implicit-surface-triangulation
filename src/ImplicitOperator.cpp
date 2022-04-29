/*
 * ImplicitOperator
 *
 * File: 	ImplicitOperator.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 23 2015
 */

#include "ImplicitOperator.hpp"

using namespace Implicit;

Operator::Operator(Object* left, Object* right) :
	m_left_child(left),
	m_right_child(right)
{
	m_iso = (left->GetIso() + right->GetIso()) / 2.;
}

Operator::Operator(Object* left, Object* right, double iso) :
	Object(iso),
	m_left_child(left),
	m_right_child(right)
{
}

glm::dvec3 Operator::GetStartVertex()
{
	return project(GetCenterVertex());
}

glm::dvec3 Operator::GetCenterVertex()
{
	return (m_left_child->GetCenterVertex()-m_right_child->GetCenterVertex())*0.5;
}
