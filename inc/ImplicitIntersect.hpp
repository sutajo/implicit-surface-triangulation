/*
 * ImplicitIntersect
 *
 * File: 	ImplicitIntersect.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 25 2015
 */

#ifndef IMPLICIT_INTERSECT_HPP
#define IMPLICIT_INTERSECT_HPP

#include "ImplicitOperator.hpp"

namespace Implicit
{
	/**
	 * \brief Gets the shape of the intersection of two implicit Objects
	 */
	class Intersect : public Operator
	{
	public:
		/**
		 * \brief Construct the intersection of two objects
		 * Averages the iso values of the two objects
		 */
		Intersect(Object* left, Object* right);

		/**
		 * \brief Construct the intersection of two objects
		 * Sets the iso value to iso
		 */
		Intersect(Object* left, Object* right, double iso);

		virtual double Evaluate(const glm::dvec3& point);
		virtual double FieldValue(const glm::dvec3& point);

		virtual glm::dvec3 Normal(const glm::dvec3& point);

	protected:

	private:
		void compute_bounds();
	};
};

#endif//IMPLICIT_INTERSECT_HPP
