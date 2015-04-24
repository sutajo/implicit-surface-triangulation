/*
 * ImplicitTransform
 *
 * File: 	ImplicitTransform.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#ifndef IMPLICIT_TRANSFORM_HPP
#define IMPLICIT_TRANSFORM_HPP

#include "ImplicitObject.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#ifdef DEBUG
#include <iostream>
#endif

namespace Implicit
{
	/**
	 * \brief Node for applying a 4x4 transformation matrix to a shape
	 *
	 * Used primarily for rotation, scaling, and translating an object
	 */
	class Transform : public Object
	{
	public:
		/**
		 * \brief Constructs a Transform Object
		 *
		 */
		Transform(Object* child, const glm::mat4& m);

		virtual float Evaluate(const glm::vec3& point);
		virtual float FieldValue(const glm::vec3& point);
		virtual glm::vec3 Normal(const glm::vec3& point)=0;
		virtual glm::vec3 GetStartVertex();
		virtual glm::vec3 GetCenterVertex();
	protected:

		/**
		 * \brief Constructs a transformation matrix node without a
		 * matrix.
		 *
		 * It is assumed that the matrix will be set within the
		 * constructor of the sub-class by calling the
		 * setWorldMatrix method.
		 */
		Transform(Object* child);

		/**
		 * \brief Sets the m_to_local and m_from_local matrix
		 */
		void setWorldMatrix(const glm::mat4& m);

		/**
		 * \brief The address of the child object
		 */
		Object* m_child;

		/**
		 * \brief Maps points from the world space to the local space
		 */
		glm::vec3 map_to(glm::vec3 world_point);

		/**
		 * \brief Maps points from the local space to the world space
		 */
		glm::vec3 map_from(glm::vec3 local_point);

	private:
		glm::mat4 m_to_local;
		glm::mat4 m_from_local;
	};
};


#endif//IMPLICIT_TRANSFORM_HPP
