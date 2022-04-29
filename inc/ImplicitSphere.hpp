#ifndef IMPLICIT_SPHERE_HPP
#define IMPLICIT_SPHERE_HPP

#include "ImplicitPrimitive.hpp"

namespace Implicit
{
	/**
	 * \brief primitive sphere object
	 */
	class Sphere : public Primitive
	{
	public:
		/**
		 * \brief Create new Sphere Primitive
		 *
		 * Creates a default Sphere with a field function
		 * The default iso value is 0.5
		 * The default radius is 1
		 *
		 * \param f The FieldFunction for the Sphere
		 */
		Sphere(FieldFunction f);

		/**
		 * \brief Create new Sphere Primitive
		 *
		 * Creates a Sphere with a defined iso value
		 * The default radius is 1
		 * \param f The FieldFunction for the Sphere
		 * \param iso The iso value where the function is defined
		 */
		Sphere(FieldFunction f, double iso);

		/**
		 * \brief Create new Sphere Primitive
		 *
		 * Creates a new Sphere with a defined FieldFunction, iso
		 * value and radius.
		 *
		 * \param f The FieldFunction for the Sphere
		 * \param iso The iso value where the surface is defined
		 * \param radius The radius of the object
		 */
		Sphere(FieldFunction f, double iso, double radius);

		/**
		 * \brief Evaluate the surface at a given point
		 *
		 * Where this evaluates to 0, the surface is defined
		 *
		 * \param point The point to evaluate
		 */
		virtual double Evaluate(const glm::dvec3& point);

		/**
		 * \brief Evaluate the field function at a given point
		 *
		 * \param point The point to evaluate
		 */
		virtual double FieldValue(const glm::dvec3& point);

		/**
		 * \brief returns a vertex on the surface
		 *
		 * Note: Test the field value to ensure that it is on the
		 * surface. This uses a numerical method to determine where the
		 * surface is, it may not actually be the surface!
		 *
		 * \return vertex
		 */
		virtual glm::dvec3 GetStartVertex();

		/**
		 * \brief returns the center vertex
		 *
		 * This will return (0, 0, 0) for all instances of
		 * Implicit::Primitive.
		 */
		virtual glm::dvec3 GetCenterVertex();

		virtual glm::dvec3 Normal(const glm::dvec3& point);

	protected:
		/**
		 * \brief Get distance to point
		 *
		 * \param pt The point to get the distance of
		 * \return The distance from the center to the point
		 */
		double getDistance(const glm::dvec3& pt);

		/**
		 * \brief Compute the bounding box of the primitive object
		 */
		void compute_bounds();
	private:
	};
};


#endif//IMPLICIT_SPHERE_HPP

