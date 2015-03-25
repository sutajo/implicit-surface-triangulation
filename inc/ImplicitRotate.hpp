#ifndef IMPLICIT_ROTATE_HPP
#define IMPLICIT_ROTATE_HPP

#include "ImplicitTransform.hpp"
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

namespace Implicit
{
	class Rotate : public Transform
	{
		public:
			/**
			 * \brief Rotate Implicit Object
			 *
			 * Rotates the implicit object around the x, y, z
			 * axes
			 *
			 * \param child Object to be rotated
			 * \param rotation amounts to rotate around each axis
			 */
			Rotate(Object* child, const glm::vec3& rotation);

			/**
			 * \brief Rotate Implicit Object
			 *
			 * Rotates the implict object around the x, y, z, axes.
			 *
			 * \param child Object to be rotated
			 * \param x Amount to rotate around the x axis
			 * \param y Amount to rotate around the y axis
			 * \param z Amount to rotate around the z axis
			 */
			Rotate(Object* child, float x, float y, float z);

			/**
			 * \brief Rotate Implicit Object
			 *
			 * Rotates the implicit object around an arbitrary axis
			 *
			 * \param axis Axis to rotate around
			 * \param angle Amount to rotate around the axis
			 */
			Rotate(Object* child, const glm::vec3& axis, float angle);
	};
};
#endif//IMPLICIT_ROTATE_HPP
