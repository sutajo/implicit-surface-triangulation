#include <glm/glm.hpp>

class Plane
{
public:
	/*
	* Construct a new object representing a plane.
	*/
	Plane(const glm::dvec3& point, const glm::dvec3& normal) : m_point(point), m_normal(normal) {}

	Plane(const glm::dvec3& point, const glm::dvec3& tangent, const glm::dvec3& bitangent) 
		: m_point(point),
		  m_normal(glm::normalize(glm::cross(tangent, bitangent)))
	{
	}

	/*
	* Determines if a point is above (i.e. in front of) a plane.
	* If offset is greater than zero, only return true for points that are also at least offset far away from the plane. 
	*/
	bool IsAbove(const glm::dvec3& point, double offset = 0.0);
private:
	const glm::dvec3& m_point;
	const glm::dvec3  m_normal;
};