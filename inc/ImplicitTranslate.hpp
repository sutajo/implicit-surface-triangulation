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

namespace Implicit
{
	class Translate : public Transform
	{
	public:
		Translate(Object* child, const glm::vec3& direction);
		Translate(Object* child, float x, float y, float z);
	};

};

#endif//IMPLICIT_TRANSLATE_HPP
