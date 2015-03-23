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
		Blend(Object* left, Object* right, float iso);


		virtual float Evaluate(const glm::vec3& point);
		virtual float FieldValue(const glm::vec3& point);

		virtual glm::vec3 Normal(const glm::vec3& point);
	protected:

	private:
	};
};

#endif // IMPLICIT_BLEND_HPP
