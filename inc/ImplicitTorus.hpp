#ifndef IMPLICIT_TORUS_HPP
#define IMPLICIT_TORUS_HPP

#include "ImplicitPrimitive.hpp"
#include "ImplicitDifferentiated.hpp"

namespace Implicit
{
	/**
	 * \brief Implicitly defined torus primitive object
	 */
	class Torus : public Differentiated<Torus>
	{
	public:
		friend class Differentiated<Torus>;
		/**
		 * \brief Construct a Torus Primitive Object
		 *
		 * Constructs a torus which sits on the xy plane.
		 * Defaults:
		 * 	Outer radius: 0.6
		 * 	Inner radius: 0.5
		 * 	iso Value: 0.0
		 * \param f Field Function
		 */
		Torus();

		/**
		 * \brief Construct a Torus Primitive Object
		 *
		 * Constructs a torus which sits on the xy plane.
		 * Defaults:
		 * 	Outer radius: 0.6
		 * 	Inner radius: 0.5
		 * \param f Field Function
		 * \param iso The iso value
		 */
		Torus(double iso);

		/**
		 * \brief Construct a Torus Primitive Object
		 *
		 * Constructs a torus which sits on the xy plane.
		 * \param f Field Function
		 * \param iso The iso value
		 * \param inner_radius The smaller radius on the inside of the
		 * torus
		 * \param outer_radius The larger radius on the outside of the
		 * torus
		 */
		Torus(double iso, double inner_radius, double outer_radius);

		virtual double Evaluate(const glm::dvec3& point);

		virtual double FieldValue(const glm::dvec3& point);

		virtual glm::dvec3 GetStartVertex();

		virtual glm::dvec3 GetCenterVertex();

		virtual glm::dvec3 Normal(const glm::dvec3& point);

	protected:
		double getDistance(const glm::dvec3& point);
		void compute_bounds();

		EquationT Equation;
		virtual void SetEquation() override;

	private:

		glm::dvec3 getNearest(const glm::dvec3& pt);

		// r
		double m_cross_radius;

		// R
		double m_center_radius;

	};
};

#endif//IMPLICIT_TORUS_HPP
