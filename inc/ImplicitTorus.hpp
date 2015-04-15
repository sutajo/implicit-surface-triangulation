#ifndef IMPLICIT_TORUS_HPP
#define IMPLICIT_TORUS_HPP

#include "ImplicitPrimitive.hpp"

namespace Implicit
{
	class Torus : public Primitive
	{
	public:
		Torus(FieldFunction f);
		Torus(FieldFunction f, float iso);
		Torus(FieldFunction f, float iso, float inner_radius, float outer_radius);

		virtual float Evaluate(const glm::vec3& point);

		virtual float FieldValue(const glm::vec3& point);

		virtual glm::vec3 GetStartVertex();

		virtual glm::vec3 GetCenterVertex();

		virtual glm::vec3 Normal(const glm::vec3& point);

	protected:
		float getDistance(const glm::vec3& point);
		void compute_bounds();
	private:

		glm::vec3 getNearest(const glm::vec3& pt);
		float m_inner_radius;
		float m_outer_radius;

		// Radius of the cross-section circle
		float m_cross_radius;

		float m_center_radius;

	};
};

#endif//IMPLICIT_TORUS_HPP
