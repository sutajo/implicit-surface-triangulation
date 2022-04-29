/*
 * ImplicitOperator
 *
 * File: 	ImplicitOperator.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 23 2015
 */

#ifndef IMPLICIT_OPERATOR_HPP
#define IMPLICIT_OPERATOR_HPP

#include "ImplicitObject.hpp"

namespace Implicit
{
	/**
	 * \brief Binary operations performed on two simpler implicit objects
	 */
	class Operator : public Object
	{
	public:
		/**
		 * \brief Creates a new binary operator
		 * Iso values are averaged between the two objects
		 * \param left Address of Object on left side
		 * \param right Address of Object on right side
		 */
		Operator(Object* left, Object* right);

		/**
		 * \brief Creates a new binary operator
		 *
		 * \param left Address of Object on left side
		 * \param right Address of Object on right side
		 * \param iso Iso value where surface exists
		 */
		Operator(Object* left, Object* right, double iso);

		virtual double Evaluate(const glm::dvec3& point)=0;
		virtual double FieldValue(const glm::dvec3& point)=0;
		virtual glm::dvec3 Normal(const glm::dvec3& point)=0;
		virtual glm::dvec3 GetStartVertex();
		virtual glm::dvec3 GetCenterVertex();
	protected:
		/**
		 * \brief Left child object
		 *
		 * This is one of the objects having the operation performed on
		 * it.
		 */
		Object* m_left_child;
		/**
		 * \brief Right child object
		 *
		 * This is one of the objects having the operation performed on
		 * it.
		 */
		Object* m_right_child;
	private:
	};
};

#endif//IMPLICIT_OPERATOR_HPP
