#include "Box.hpp"
#include <vector>

Box::Box(float w, float h, float d)
{
	Resize(w, h, d);
}

void Box::Resize(float w, float h, float d)
{
	std::vector<Vertex> geometry = {
		{ glm::vec3(-w, -h, -d), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w,  h, -d), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f) },
		{ glm::vec3(w, -h, -d), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f) },

		{ glm::vec3(-w,  h, -d), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f) },
		{ glm::vec3(w,  h, -d), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f) },
		{ glm::vec3(w, -h, -d), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f) },

		{ glm::vec3(w, -h, -d), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w,  h, -d), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w, -h,  d), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },

		{ glm::vec3(w,  h, -d), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w,  h,  d), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w, -h,  d), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },

		{ glm::vec3(w,  -h, d), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f) },
		{ glm::vec3(w,   h, d), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w, -h, d), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f) },

		{ glm::vec3(w,  h,  d), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w, h,  d), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w,-h,  d), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f) },

		{ glm::vec3(-w, -h, d), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w,  h,-d), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w, -h,-d), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },

		{ glm::vec3(-w,  h, d), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w,  h,-d), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w, -h, d), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f) },

		{ glm::vec3(-w, -h,-d), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w, -h,-d), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w, -h, d), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f) },

		{ glm::vec3(-w, -h, d), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w, -h,-d), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w, -h, d), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f) },

		{ glm::vec3(-w,  h, d), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w,  h, d), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(-w,  h,-d), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f) },

		{ glm::vec3(-w,  h,-d), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w,  h, d), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f) },
		{ glm::vec3(w,  h,-d), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f) },
	};

	Update(geometry, center);
}

void Box::SetCenter(const glm::vec3& center)
{
	this->center = center;
}

Box::~Box()
{
}
