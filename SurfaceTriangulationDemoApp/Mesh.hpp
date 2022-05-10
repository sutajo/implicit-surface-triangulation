#pragma once

#include <OpenMesh/Core/IO/MeshIO.hh>
#include "glmMeshAdaptor.hpp"

#include <GL/glew.h>
#include <glm/gtc/quaternion.hpp>
#include <vector>

#include "Common/Camera.hpp"

enum class FaceVisualization : int
{
	Id = 0,
	Normal = 1,
	EarCuttingFaces = 2
};

struct Vertex
{
	glm::vec3 Position;
	glm::vec3 Normal;
	glm::vec3 Color;
};

class Mesh
{
private:
	GLuint vertexArray;
	GLuint vertexBuffer;

	size_t numVertices = 0;

	glm::vec3 center;
	glm::mat4 worldMatrix;
	glm::quat orientation{ glm::vec3(0.0, 0.0, 0.0) };

	glm::vec3 MapFaceCreationMethodColor(FaceCreationMethod method) const;

	static glm::vec3 FindCenter(const std::vector<Vertex>& vertices);
	void UpdateWorldMat();
public:
	Mesh();
	virtual ~Mesh();

	void Render();
	void RenderAsLines();
	void Update(const std::vector<Vertex>& vertices, glm::vec3 center);
	void Update(const std::vector<Vertex>& vertices);
	std::vector<Vertex> GetMeshVertices(const GlmPolyMesh& mesh, FaceVisualization visualization = FaceVisualization::Id) const;
	std::vector<Vertex> GetLineVertices(const GlmPolyMesh& mesh) const;
	const glm::mat4& GetWorldMat();
	void RotateInCameraSpace(int relx, int rely, const Camera& camera);
	void ResetRotation() { orientation = glm::vec3(0.0, 0.0, 0.0); UpdateWorldMat(); }

	glm::vec3 GetColorFromFaceIndex(int idx) const;
	int GetFaceIndexFromColor(unsigned char r, unsigned char g, unsigned char b) const;

	size_t GetVertexCount() { return numVertices; }
	size_t GetTriangleCount() { return numVertices / 3; }

};