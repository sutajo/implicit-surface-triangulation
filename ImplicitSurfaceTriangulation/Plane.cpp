#include "Plane.hpp"

bool Plane::IsAbove(const glm::dvec3& point, double offset) const
{
    return glm::dot(point - m_point, m_normal) >= offset;
}
