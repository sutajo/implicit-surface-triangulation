/*
 * ImplicitRotate
 *
 * File: 	ImplicitRotate.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 25 2015
 */

#ifndef IMPLICIT_ROTATE_HPP
#define IMPLICIT_ROTATE_HPP

#include "ImplicitTransform.hpp"

#include <glm/gtc/matrix_transform.hpp>

namespace Implicit
{
	/**
	 * \brief Rotates the implicit object
	 */
	class Rotate : public Transform
	{
		public:
			/**
			 * \brief Rotate Implicit Object
			 *
			 * Rotates the implicit object around an arbitrary axis
			 *
			 * \param child The original, un-rotated object
			 * \param axis The arbitrary axis to rotate around --
			 * Normalized within the constructor
			 * \param angle The amount to rotate around the axis,
			 * in radians
			 */
			Rotate(Object* child, const glm::dvec3& axis, double angle);
			virtual glm::dvec3 Normal(const glm::dvec3& point);

	};
};
#endif//IMPLICIT_ROTATE_HPP
