#ifndef IMPLICIT_LINE_HPP
#define IMPLICIT_LINE_HPP

#include "ImplicitPrimitive.hpp"
namespace Implicit
{
	class Line : public Primitive
	{
	public:
		Line(FieldFunction f);
		Line(FieldFunction f, float iso);
		Line(FieldFunction f, float iso, float radius);
		Line(FieldFunction f, float iso, float radius, float length);

		/**
		 * \brief Evaluate the surface at a given point
		 *
		 * Where this evaluates to 0, the surface is defined
		 *
		 * \param point The point to evaluate
		 */
		virtual float Evaluate(const glm::vec3& point);

		/**
		 * \brief Evaluate the field function at a given point
		 *
		 * \param point The point to evaluate
		 */
		virtual float FieldValue(const glm::vec3& point);

		/**
		 * \brief returns a vertex on the surface
		 *
		 * Note: Test the field value to ensure that it is on the
		 * surface. This uses a numerical method to determine where the
		 * surface is, it may not actually be the surface!
		 *
		 * \return vertex
		 */
		virtual glm::vec3 GetStartVertex();

		/**
		 * \brief returns the center vertex
		 *
		 * This will return (0, 0, 0) for all instances of
		 * Implicit::Primitive.
		 */
		virtual glm::vec3 GetCenterVertex();

		virtual glm::vec3 Normal(const glm::vec3& point);

	protected:

		void compute_bounds();

	private:

		glm::vec3 getNearest(const glm::vec3& point);
		float getDistance(const glm::vec3& point);

		float m_length;
		glm::vec3 m_endpoint_1;
		glm::vec3 m_endpoint_2;

	};
};

#endif//IMPLICIT_LINE_HPP
