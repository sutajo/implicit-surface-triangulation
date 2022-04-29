/*
 * Aabb
 *
 * File: Aabb.hpp
 * Auth: Evan Wilde					<etcwilde@uvic.ca>
 * Date: Mar 22 2015
 */

#ifndef AABB_HPP
#define AABB_HPP


#define AABB_RAY_CACHE_SIZE	511
#define RAY_HASH_MULTIPLIER	0xf4243

#include <vector>
#include <list>
#include <functional>

#include <glm/glm.hpp>
#include <string.h>

#include <iostream>
#include "VecPrint.hpp"

/**
 * AABB -- Axis-aligned bounding box
 * Use for Bounding Volume Hierarchies
 */
class Aabb
{

public:
	enum Axis
	{
		X,
		Y,
		Z
	};


	Aabb() { reset(); }

	/**
	 * Reset AABB to default state
	 */
	void reset();

	/**
	 * Update AABB to include a given point
	 */
	void include(const glm::dvec3& point);

	/**
	 * Computes the AABB of a set of vertices.
	 * Overwrites this AABB
	 */
	void compute(const std::vector<glm::dvec3>& verts);

	/**
	 * generates the AABB over a list of vertices
	 * Overwrites current AABB */
	void compute(std::list<glm::dvec3>& verts);

	/**
	 * Union this AABB with another AABB
	 * Overwrites this AABB
	 */
	void add(const Aabb& other);

	/**
	 * Expand the AABB uniformly by a given factor
	 */
	void expand(double factor);

	/**
	 * Transform the AABB
	 */
	void transform(const glm::dmat4& transform);

	/**
	 * Get the volume of the AABB
	 */
	double volume() const;

	/**
	 * Get the surface area of the AABB
	 */
	double surfaceArea() const;

	/**
	 * Get the length of the longest side
	 */
	double size() const;

	/**
	 * \brief Determine if the AABB overlaps another
	 */
	bool overlap(const Aabb& other) const;

	/**
	 * \brief Determines if a point is in the bounding box
	 * \param point point to test
	 * \return if the point is within the bounding box
	 */
	bool contains(const glm::dvec3& point) const;



	/**
	 * \brief Determines if a ray intersects the AABB
	 *
	 * Designed for ray tracing purposes
	 * \param origin origin of the ray
	 * \param direction direction the ray is pointing
	 * \param hit_point location of closest collision
	 * \return if ray intersects aabb
	 *
	 * Note: If the origin of the ray is within the box, the hit_point will
	 * be the origin
	 */
	bool intersect(const glm::dvec3& origin, const glm::dvec3& direction, glm::dvec3& hit_point);


	/**
	 * \brief Returns the minimum bounding coordinate
	 *
	 * \return Minimum bounding coordinate in 3D space
	 */
	const glm::dvec3& min() const;

	/**
	 * \brief Returns the maximum bounding coordinate
	 *
	 * \return Maximum bounding coordinate in 3D space
	 */
	const glm::dvec3& max() const;

	Aabb::Axis majorAxis() const;

	Aabb::Axis minorAxis() const;

private:
	class AABB_RAY
	{
		public:
			AABB_RAY() {}
			AABB_RAY(const glm::dvec3& o, const glm::dvec3& d) :
				origin(o),
				direction(d)
			{
				inv_direction = glm::dvec3(1/d.x, 1/d.y, 1/d.z);
				sign[0] = (inv_direction.x < 0);
				sign[1] = (inv_direction.y < 0);
				sign[2] = (inv_direction.z < 0);
			}

			glm::dvec3 origin;
			glm::dvec3 direction;
			glm::dvec3 inv_direction;
			int sign[3];
	};

	/*
	 * Algorithm taken from Python Tuple Hash
	 * #define _PyHASH_MULTIPLIER 1000003
	 * static Py_hash_t
	 * tuplehash(PyTupleObject *v)
	 * {
	 *
	 * 	Py_uhash_t x;  // Unsigned for defined overflow behavior.
	 * 	Py_hash_t y;
	 * 	Py_ssize_t len = Py_SIZE(v);
	 * 	PyObject **p;
	 * 	Py_uhash_t mult = _PyHASH_MULTIPLIER;
	 * 	x = 0x345678UL;
	 * 	p = v->ob_item;
	 * 	while (--len >= 0) {
	 * 		y = PyObject_Hash(*p++);
	 * 		if (y == -1)
	 * 			return -1;
	 * 		x = (x ^ y) * mult;
	 * 		//Cast may truncate len; doesn't change stability
	 * 		mult += (Py_hash_t)(82520UL + len + len);
	 * 	}
	 * 	x += 97531UL;
	 * 	if (x == (Py_uhash_t)-1)
	 * 		x = -2;
	 * 	return x;
	 * }
	 *
	 * -- Simplified for six-tuple origin(x, y, z), direction(x, y, z)
	 */
	static unsigned int hash_ray(const glm::dvec3& origin, const glm::dvec3& direction);
	static unsigned int hash_ray(const AABB_RAY& R);

	AABB_RAY m_ray_cache[AABB_RAY_CACHE_SIZE];

	glm::dvec3 bounds[2];
};

#endif//AABB_HPP
