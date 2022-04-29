#ifndef IMPLICIT_LINE_HPP
#define IMPLICIT_LINE_HPP

#include "ImplicitPrimitive.hpp"
namespace Implicit
{
	/**
	 * \brief A Line is a straight line with rounded ends
	 *
	 * Lines are straight segments of a defined length, with a given
	 * maximum radius and iso value.
	 */
	class Line : public Primitive
	{
	public:
		/**
		 * \brief Construct a Line Primitive Object
		 *
		 * The shape is unit length, centered around local (0, 0, 0),
		 * with an iso value of 0.5, with a maximum radius 1.
		 *
		 * \param f FieldFunction for the shape
		 */
		Line(FieldFunction f);

		/**
		 * \brief Construct a Line Primitive Object
		 *
		 * The shape is unit length, centered around local (0, 0, 0).
		 *
		 * \param f FieldFunction for the shape
		 * \param iso The iso value of the shape
		 */
		Line(FieldFunction f, double iso);

		/**
		 * \brief Construct a Line Primitive Object
		 *
		 * The shape is unit length, centered around local (0, 0, 0).
		 *
		 * \param f FieldFunction for the shape
		 * \param iso The iso value of the shape
		 * \param radius The defined maximum radius of the shape
		 */
		Line(FieldFunction f, double iso, double radius);

		/**
		 * \brief Construct a Line Primitive Object
		 *
		 * The line is centered around the local coordinate (0, 0, 0).
		 * 		 *
		 * \param f FieldFunction for the shape
		 * \param iso The iso value of the shape
		 * \param radius The defined maximum radius of the shape
		 * \param length The lenght of the line
		 */
		Line(FieldFunction f, double iso, double radius, double length);


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

		void compute_bounds();

	private:

		glm::dvec3 getNearest(const glm::dvec3& point);
		double getDistance(const glm::dvec3& point);

		double m_length;
		glm::dvec3 m_endpoint_1;
		glm::dvec3 m_endpoint_2;

	};
};

#endif//IMPLICIT_LINE_HPP
