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
	class Rotate : public Transform
	{
		public:
			/**
			 * \brief Rotate Implicit Object
			 *
			 * Rotates the implicit object around an arbitrary axis
			 *
			 * \param axis Axis to rotate around
			 * \param angle Amount to rotate around the axis
			 */
			Rotate(Object* child, const glm::vec3& axis, float angle);
			virtual glm::vec3 Normal(const glm::vec3& point);

	};
};
#endif//IMPLICIT_ROTATE_HPP
