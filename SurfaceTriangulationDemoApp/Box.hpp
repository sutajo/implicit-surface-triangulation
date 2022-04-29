#pragma once

#include "Mesh.hpp"

class Box : public Mesh
{
private:
	glm::vec3 center{0.0f};

public:
	Box(float w, float h, float d);
	virtual ~Box();

	void Resize(float w, float h, float d);
	void SetCenter(const glm::vec3& center);
};