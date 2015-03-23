#include "ImplicitOperator.hpp"

using namespace Implicit;

Operator::Operator(Object* left, Object* right) :
	m_left_child(left),
	m_right_child(right)
{
	m_iso = (left->GetIso() + right->GetIso()) / 2.f;
}

Operator::Operator(Object* left, Object* right, float iso) :
	Object(iso),
	m_left_child(left),
	m_right_child(right)
{ }
