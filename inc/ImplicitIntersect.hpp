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
	class Intersect : public Operator
	{
		/**
		 * \brief Construct the intersection of two objects
		 * Averages the iso values of the two objects
		 */
		Intersect(Object* left, Object* right);

		Intersect(Object* left, Object* right, float iso);

		virtual float Evaluate(const glm::vec3& point);
		virtual float FieldValue(const glm::vec3& point);

		virtual glm::vec3 Normal(const glm::vec3& point);

	protected:

	private:

	};
};

#endif//IMPLICIT_INTERSECT_HPP
