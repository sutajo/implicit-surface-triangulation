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
		Union(Object* left, Object* right, float iso);

		virtual float Evaluate(const glm::vec3& point);
		virtual float FieldValue(const glm::vec3& point);

		virtual glm::vec3 Normal(const glm::vec3& point);
	};
};

#endif//IMPLICIT_UNION_HPP
