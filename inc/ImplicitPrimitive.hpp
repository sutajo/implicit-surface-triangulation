/*
 * ImplicitPrimitive
 *
 * File: 	ImplicitPrimitive.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Feb 26 2015
 */

#ifndef IMPLICIT_PRIMITIVE_HPP
#define IMPLICIT_PRIMITIVE_HPP

#include "ImplicitObject.hpp"

#include <glm/glm.hpp>

#ifdef DEBUG
#include <iostream>
#endif

namespace Implicit
{

	// First float is the current radius, second float is max radius of
	// function
	/**
	 * \brief Field functions determine how the blob interacts with other
	 * blobs
	 *
	 * This function is evaluated at a distance r, and a maximum distance
	 * R. If r exceeds the maximum distance R, then the function should
	 * evaluate to 0.
	 *
	 * \param r distance from skeleton of the primitive
	 * \param R maximum distance from skeleton
	 */
	typedef double(*FieldFunction)(double r, double R);

	/**
	 * \brief A blob with a defined field function and iso value
	 *
	 * A primitive is currently just a blob.
	 * A blob is defined by a central point and a field falloff function
	 * surrounding the point. Where the field falloff function evaluates to
	 * a given iso value, the surface is defined.
	 */
	class Primitive : public Object
	{
	public:
		/**
		 * \brief Create new primitive
		 *
		 * Creates a default primitive with a field function
		 * The default iso value is 0.5
		 * The default radius is 1
		 *
		 * \param f The FieldFunction for the Primitive
		 */
		Primitive(FieldFunction f);

		/**
		 * \brief Create new Primitive
		 *
		 * Creates a Primitive with a defined iso value
		 * The default radius is 1
		 * \param f The FieldFunction for the primitive
		 * \param iso The iso value where the function is defined
		 */
		Primitive(FieldFunction f, double iso);

		/**
		 * \brief Create new Primitive
		 *
		 * Creates a new Primitive with a defined FieldFunction, iso
		 * value and radius.
		 *
		 * \param f The FieldFunction for the Primitive
		 * \param iso The iso value where the surface is defined
		 * \param radius The radius of the object
		 */
		Primitive(FieldFunction f, double iso, double radius);



		/**
		 * \brief Evaluate the surface at a given point
		 *
		 * Where this evaluates to 0, the surface is defined
		 *
		 * \param point The point to evaluate
		 */
		virtual double Evaluate(const glm::dvec3& point)=0;

		/**
		 * \brief Evaluate the field function at a given point
		 *
		 * \param point The point to evaluate
		 */
		virtual double FieldValue(const glm::dvec3& point)=0;

		/**
		 * \brief returns a vertex on the surface
		 *
		 * Note: Test the field value to ensure that it is on the
		 * surface. This uses a numerical method to determine where the
		 * surface is, it may not actually be the surface!
		 *
		 * \return vertex
		 */
		virtual glm::dvec3 GetStartVertex()=0;

		/**
		 * \brief returns the center vertex
		 *
		 * This will return (0, 0, 0) for all instances of
		 * Implicit::Primitive.
		 */
		virtual glm::dvec3 GetCenterVertex()=0;

		virtual glm::dvec3 Normal(const glm::dvec3& point)=0;

	protected:

		/**
		 * \brief Evaluates the field function at a distance
		 *
		 * Will return a value between 0 and 1
		 * \return The result of the field function at distance r
		 */
		virtual double FieldValue(double r);

		/**
		 * \brief Evaluate the surface at a distance
		 *
		 * Where this evaluates to 0, the surface is defined
		 *
		 * \param r The distance to evaluate the function
		 */
		virtual double Evaluate(double r);


		/**
		 * \brief Get distance to point
		 *
		 * \param pt The point to get the distance of
		 * \return The distance from the center to the point
		 */
		virtual double getDistance(const glm::dvec3& pt)=0;

		/**
		 * \brief Compute the bounding box of the primitive object
		 */
		virtual void compute_bounds()=0;

		/**
		 * \brief Field falloff function of the primitive blob
		 */
		FieldFunction m_fieldFunction;
		/**
		 * \brief Maximum radius of the primitive
		 */
		double m_radius;
	private:
	};
};

#endif // IMPLICIT_PRIMITIVE_HPP
