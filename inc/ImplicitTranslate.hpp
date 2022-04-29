/*
 * ImplicitTranslate
 *
 * File: 	ImplicitTranslate.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 23 2015
 */

#ifndef IMPLICIT_TRANSLATE_HPP
#define IMPLICIT_TRANSLATE_HPP

#include "ImplicitTransform.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace Implicit
{
	/**
	 * \brief Translates the object represented by the child through space
	 */
	class Translate : public Transform
	{
	public:
		/**
		 * \brief Construct a translation node
		 *
		 * \param child Object to be translated
		 * \param direction The vector to translate the object along.
		 * The magnitude of the vector defines how far.
		 */
		Translate(Object* child, const glm::dvec3& direction);

		/**
		 * \brief Construct a translation
		 *
		 * \param child Object to be translated
		 * \param x Amount to translate along the x axis
		 * \param y Amount to translate along the y axis
		 * \param z Amount to translate along the z axis
		 */
		Translate(Object* child, double x, double y, double z);
		virtual glm::dvec3 Normal(const glm::dvec3& point);
	};

};

#endif//IMPLICIT_TRANSLATE_HPP
