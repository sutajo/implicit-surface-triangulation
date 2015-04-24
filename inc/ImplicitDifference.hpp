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
	/**
	 * \brief Subtracts one implicit object from another
	 */
	class Difference : public Operator
	{
	public:
		/**
		 * \brief Construct a Difference Object taking the difference
		 * of the two objects
		 *
		 * \param left The object having volume removed
		 * \param right The object removing the volume
		 */
		Difference(Object* left, Object* right);

		/**
		 * \brief Construct a Difference Object taking the difference
		 * of the two objects
		 *
		 * \param left The object having volume removed
		 * \param right The object removing the volume
		 * \param iso The iso value for the operation
		 */
		Difference(Object* left, Object* right, float iso);

		virtual float Evaluate(const glm::vec3& point);
		virtual float FieldValue(const glm::vec3& point);

		virtual glm::vec3 Normal(const glm::vec3& point);
	private:
		void compute_bounds();
	};
};

#endif//IMPLICIT_DIFFERENCE_HPP
