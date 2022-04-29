/*
 * ImplicitBlend
 *
 * File: 	ImplicitBlend.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 06 2015
 */

#ifndef IMPLICIT_BLEND_HPP
#define IMPLICIT_BLEND_HPP

#include "ImplicitOperator.hpp"

#ifdef DEBUG
#include <iostream>
#endif

namespace Implicit
{
	/**
	 * \brief Blends multiple primitive objects into one blobject
	 */
	class Blend : public Operator
	{
	public:
		/**
		 * \brief Construct a Blend Object
		 * Averages the iso values of the two objects
		 */
		Blend(Object* left, Object* right);
		/**
		 * \brief Constructs a Blend Object
		 */
		Blend(Object* left, Object* right, double iso);


		virtual double Evaluate(const glm::dvec3& point);
		virtual double FieldValue(const glm::dvec3& point);

		virtual glm::dvec3 Normal(const glm::dvec3& point);
	protected:

	private:
		void compute_bounds();
	};
};

#endif // IMPLICIT_BLEND_HPP
