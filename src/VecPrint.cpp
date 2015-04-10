#include "VecPrint.hpp"
std::ostream& operator<<(std::ostream& os, const glm::vec3& v)
{
		os << "[" << v.x << ", " << v.y << ", " << v.z << "]";
			return os;
}

