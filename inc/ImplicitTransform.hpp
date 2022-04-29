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
		Transform(Object* child, const glm::dmat4& m);

		virtual double Evaluate(const glm::dvec3& point);
		virtual double FieldValue(const glm::dvec3& point);
		virtual glm::dvec3 Normal(const glm::dvec3& point)=0;
		virtual glm::dvec3 GetStartVertex();
		virtual glm::dvec3 GetCenterVertex();
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
		void setWorldMatrix(const glm::dmat4& m);

		/**
		 * \brief The address of the child object
		 */
		Object* m_child;

		/**
		 * \brief Maps points from the world space to the local space
		 */
		glm::dvec3 map_to(glm::dvec3 world_point);

		/**
		 * \brief Maps points from the local space to the world space
		 */
		glm::dvec3 map_from(glm::dvec3 local_point);

	private:
		glm::dmat4 m_to_local;
		glm::dmat4 m_from_local;
	};
};


#endif//IMPLICIT_TRANSFORM_HPP
