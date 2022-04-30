#include "Mesh.hpp"
#include <numeric>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>
#include <functional>

Mesh::Mesh() : worldMatrix(1.0f)
{
	glGenVertexArrays(1, &vertexArray);
	glBindVertexArray(vertexArray);

	glGenBuffers(1, &vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void*) offsetof(Vertex, Position));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void*) offsetof(Vertex, Normal));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer((GLuint)2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void*) offsetof(Vertex, Color));

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

Mesh::~Mesh()
{
	glDeleteBuffers(1, &vertexBuffer);
	glDeleteVertexArrays(1, &vertexArray);
}

void Mesh::Render()
{
	glBindVertexArray(vertexArray);
	glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(numVertices));
	glBindVertexArray(0);
}

glm::vec3 Mesh::MapFaceCreationMethodColor(FaceCreationMethod method) const
{
	if (method == FaceCreationMethod::Seed)
		return glm::vec3(1.0f, 0.3f, 0.0f);
	else if (method == FaceCreationMethod::IsoscelesGrowing)
		return glm::vec3(0.0f, 1.0f, 0.0f);
	else if (method == FaceCreationMethod::EarCutting)
		return glm::vec3(1.0f, 1.0f, 0.0f);
}

glm::vec3 Mesh::FindCenter(const std::vector<Vertex>& vertices)
{
	glm::vec3 center = std::accumulate(vertices.begin(), vertices.end(), glm::vec3(0.0f), [](const glm::vec3& acc, const Vertex& v) { return acc + v.Position; });
	center /= vertices.size();
	return center;
}

void Mesh::UpdateWorldMat()
{
	worldMatrix = glm::translate(-center) * glm::toMat4(orientation);
}

void Mesh::Update(const std::vector<Vertex>& vertices, glm::vec3 center)
{
	this->center = center;
	UpdateWorldMat();

	numVertices = vertices.size();

	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Mesh::Update(const std::vector<Vertex>& vertices)
{
	Update(vertices, FindCenter(vertices));
}

std::vector<Vertex> Mesh::GetMeshVertices(const GlmMesh& mesh, FaceVisualization visualization) const
{
	std::vector<Vertex> vertices(mesh.n_faces() * 3);
	size_t index = 0;
	for (auto f : mesh.faces())
		for (auto v : mesh.fv_range(f))
		{
			vertices[index].Position = mesh.point(v);
			vertices[index].Normal = mesh.normal(v);
			vertices[index].Color = 
				visualization == FaceVisualization::Id ? 
					GetColorFromFaceIndex(f.idx()) 
				: ( 
					visualization == FaceVisualization::Normal ?
						glm::vec3((mesh.normal(v) / 2. + 1.))
					:
					 	MapFaceCreationMethodColor( mesh.data(f).faceCreationMethod )
				  );
			++index;
		}

	return vertices;
}

const glm::mat4& Mesh::GetWorldMat()
{
	return worldMatrix;
}

void Mesh::RotateInCameraSpace(int relx, int rely, const Camera& camera)
{
	// Forgatás kamera koordinátarendszerben
	// https://nicolbolas.github.io/oldtut/Positioning/Tut08%20Camera%20Relative%20Orientation.html
	auto offset = glm::quat(glm::vec3(
		glm::radians((float)rely),
		glm::radians((float)relx),
		0.0f
	));

	
	const glm::vec3& camPos = camera.GetEye();
	const glm::mat4& camMat = camera.GetViewMatrix();

	const glm::fquat viewQuat = glm::quat_cast(camMat);
	const glm::fquat invViewQuat = glm::conjugate(viewQuat);

	const glm::fquat worldQuat = (invViewQuat * offset * viewQuat);
	orientation = worldQuat * orientation;

	UpdateWorldMat();
}

glm::vec3 Mesh::GetColorFromFaceIndex(int idx) const
{
	auto color = (idx * 3) / 3;
	return glm::vec3(1.f - (color & 0xFF) / 255.f, 1.f - ((color >> 8) & 0xFF) / 255.f, 1.f - ((color >> 16) & 0xFF) / 255.f);
}

int Mesh::GetFaceIndexFromColor(unsigned char r, unsigned char g, unsigned char b) const
{
	const int roffset = 255-r;
	const int goffset = (255-g) << 8;
	const int boffset = (255-b) << 16;
	int idx = boffset | goffset | roffset;
	if (idx >= numVertices / 3)
		idx = -1;
	return idx;
}