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
	class Transform : public Object
	{
	public:
		Transform(Object* child, const glm::mat4& m);


		virtual float Evaluate(const glm::vec3& point);
		virtual float FieldValue(const glm::vec3& point);
		virtual glm::vec3 Normal(const glm::vec3& point);
		virtual glm::vec3 GetStartVertex();
		virtual glm::vec3 GetCenterVertex();

	protected:
		Transform(Object* child);
		void setWorldMatrix(const glm::mat4& m);
		Object* m_child;
	private:
		glm::vec3 map_to(glm::vec3 world_point);
		glm::vec3 map_from(glm::vec3 local_point);
		glm::mat4 m_to_local;
		glm::mat4 m_from_local;
		glm::mat4 m_normal_conversion;
	};
};


#endif//IMPLICIT_TRANSFORM_HPP
