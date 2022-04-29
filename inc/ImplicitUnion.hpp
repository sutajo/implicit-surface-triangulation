/*
 * ImplicitUnion
 *
 * File: 	ImplicitUnion.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#ifndef IMPLICIT_UNION_HPP
#define IMPLICIT_UNION_HPP
#include <algorithm>

#include "ImplicitOperator.hpp"

namespace Implicit
{
	/**
	 * \brief Unions two objects together
	 *
	 * The field functions do not effect each-other.
	 */
	class Union : public Operator
	{
	public:
		/**
		 * \brief Construct a Union Object
		 * Averages the iso value of the two objects
		 */
		Union(Object* left, Object* right);

		/**
		 * \brief Constructs a Union Object
		 * Sets the iso value to iso
		 */
		Union(Object* left, Object* right, double iso);

		virtual double Evaluate(const glm::dvec3& point);
		virtual double FieldValue(const glm::dvec3& point);

		virtual glm::dvec3 Normal(const glm::dvec3& point);
	private:
		void compute_bounds();

	};
};

#endif//IMPLICIT_UNION_HPP
