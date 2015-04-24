#ifndef IMPLICIT_TORUS_HPP
#define IMPLICIT_TORUS_HPP

#include "ImplicitPrimitive.hpp"

namespace Implicit
{
	/**
	 * \brief Implicitly defined torus primitive object
	 */
	class Torus : public Primitive
	{
	public:
		/**
		 * \brief Construct a Torus Primitive Object
		 *
		 * Constructs a torus which sits on the xy plane.
		 * Defaults:
		 * 	Outer radius: 0.6
		 * 	Inner radius: 0.5
		 * 	iso Value: 0.5
		 * \param f Field Function
		 */
		Torus(FieldFunction f);

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
		Torus(FieldFunction f, float iso);

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
