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
		Intersect(Object* left, Object* right, float iso);

		virtual float Evaluate(const glm::vec3& point);
		virtual float FieldValue(const glm::vec3& point);

		virtual glm::vec3 Normal(const glm::vec3& point);

	protected:

	private:
		void compute_bounds();
	};
};

#endif//IMPLICIT_INTERSECT_HPP
