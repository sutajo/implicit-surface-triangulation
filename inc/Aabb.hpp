#ifndef AABB_HPP
#define AABB_HPP

#include <vector>
#include <list>

#include <glm/glm.hpp>

#include <iostream>



#ifdef DEBUG



#endif


/**
 * AABB -- Axis-aligned bounding box
 * Use for Bounding Volume Hierarchies
 */
class Aabb
{

public:
	Aabb() { reset(); }

	/**
	 * Reset AABB to default state
	 */
	void reset();

	/**
	 * Update AABB to include a given point
	 */
	void include(const glm::vec3& point);

	/**
	 * Computes the AABB of a set of vertices.
	 * Overwrites this AABB
	 */
	void compute(const std::vector<glm::vec3>& verts);


	/**
	 * generates the AABB over a list of vertices
	 * Overwrites current AABB
	 */
	void compute(std::list<glm::vec3>& verts);

	/**
	 * Union this AABB with another AABB
	 * Overwrites this AABB
	 */
	void add(const Aabb& other);

	/**
	 * Expand the AABB uniformly by a given factor
	 */
	void expand(float factor);

	/**
	 * Transform the AABB
	 */
	void transform(const glm::mat4& transform);

	/**
	 * Get the volume of the AABB
	 */
	float volume() const;

	/**
	 * Get the surface area of the AABB
	 */
	float surfaceArea() const;

	/**
	 * Get the length of the longest side
	 */
	float size() const;

	/**
	 * \brief Determine if the AABB overlaps another
	 */
	bool overlap(const Aabb& other) const;

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
	bool intersect(const glm::vec3& origin, const glm::vec3& direction, glm::vec3& hit_point);

	glm::vec3 m_maxima;
	glm::vec3 m_minima;
};

#endif//AABB_HPP
