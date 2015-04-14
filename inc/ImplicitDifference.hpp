/*
 * ImplicitDifference
 *
 * File: 	ImplicitDifference.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Apr 14 2015
 */

#ifndef IMPLICIT_DIFFERENCE_HPP
#define IMPLICIT_DIFFERENCE_HPP
#include <algorithm>

#include "ImplicitOperator.hpp"

namespace Implicit
{
	class Difference : public Operator
	{
	public:
		Difference(Object* left, Object* right);
		Difference(Object* left, Object* right, float iso);

		virtual float Evaluate(const glm::vec3& point);
		virtual float FieldValue(const glm::vec3& point);

		virtual glm::vec3 Normal(const glm::vec3& point);
	private:
		void compute_bounds();
	};
};

#endif//IMPLICIT_DIFFERENCE_HPP
