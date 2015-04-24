#ifndef IMPLICIT_SPHERE_HPP
#define IMPLICIT_SPHERE_HPP

#include "ImplicitPrimitive.hpp"

namespace Implicit
{

	class Sphere : public Primitive
	{
	public:
		Sphere(FieldFunction f);
		Sphere(FieldFunction f, float iso);
		Sphere(FieldFunction f, float iso, float radius);

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
		/**
		 * \brief Get distance to point
		 *
		 * \param pt The point to get the distance of
		 * \return The distance from the center to the point
		 */
		float getDistance(const glm::vec3& pt);

		/**
		 * \brief Compute the bounding box of the primitive object
		 */
		void compute_bounds();
	private:
	};
};


#endif//IMPLICIT_SPHERE_HPP

