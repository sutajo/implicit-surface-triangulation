#include "Mesh.hpp"
#include <numeric>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>
#include <functional>

using namespace glm;
using namespace std;
using namespace Implicit::Tessellation;

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

void Mesh::RenderAsLines()
{
	glBindVertexArray(vertexArray);
	glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(numVertices));
	glBindVertexArray(0);
}

vec3 Mesh::MapFaceCreationMethodColor(FaceCreationMethod method) const
{
	switch (method)
	{
	case FaceCreationMethod::Seed:
		return vec3(1.0f, 0.3f, 0.0f);
	case FaceCreationMethod::IsoscelesGrowing:
		return vec3(0.0f, 1.0f, 0.0f);
	case FaceCreationMethod::EarCutting:
		return vec3(1.0f, 1.0f, 0.0f);
	case FaceCreationMethod::SmallPolygonFilling:
		return vec3(140 / 255.f, 252 / 255.f, 3 / 255.f);
	case FaceCreationMethod::XFilling:
		return vec3(1.0f, 140.0f / 255.0f, 0.0f);
	case FaceCreationMethod::EarFilling:
		return vec3(244 / 255.f, 3 / 255.f, 252 / 255.f);
	case FaceCreationMethod::ConvexPolygonFilling:
		return vec3(3 / 255.f, 252 / 255.f, 144 / 255.f);
	case FaceCreationMethod::RelaxedEarFilling:
		return vec3(3 / 255.f, 252 / 255.f, 252 / 255.f);
	case FaceCreationMethod::ConcaveVertexBisection:
		return vec3(3 / 255.f, 61 / 255.f, 252 / 255.f);
	case FaceCreationMethod::SubdivisionOnBridges:
	default:
		throw runtime_error("Unhandled method");
	}	
}

vec3 Mesh::FindCenter(const vector<Vertex>& vertices)
{
	vec3 center = accumulate(vertices.begin(), vertices.end(), vec3(0.0f), [](const vec3& acc, const Vertex& v) { return acc + v.Position; });
	center /= vertices.size();
	return center;
}

void Mesh::UpdateWorldMat()
{
	worldMatrix = translate(-center) * toMat4(orientation);
}

void Mesh::Update(const vector<Vertex>& vertices, vec3 center)
{
	this->center = center;
	UpdateWorldMat();

	numVertices = vertices.size();

	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Mesh::Update(const vector<Vertex>& vertices)
{
	Update(vertices, FindCenter(vertices));
}

vector<Vertex> Mesh::GetMeshVertices(const GlmPolyMesh& mesh, FaceVisualization visualization) const
{
	vector<Vertex> vertices(mesh.n_faces() * 3);
	size_t index = 0;
	for (auto f : mesh.faces())
		if(f.valence() == 3)
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

vector<Vertex> Mesh::GetLineVertices(const GlmPolyMesh& mesh, const ClosestNeighbours& closestNeighbours) const
{
	vector<Vertex> vertices;
	vertices.reserve(mesh.n_edges());

	for (auto f : mesh.faces())
		if (f.valence() > 3 && !mesh.status(f).deleted())
		{
			auto heh_start = f.halfedge();
			auto heh = heh_start;

			auto face_normal = mesh.calc_face_normal(f);

			do
			{
				auto edge_vector = mesh.calc_edge_vector(heh);
				auto offset_vector = normalize(cross(face_normal, edge_vector)) * 0.0015;

				// Edge
				{
					auto edgeColorRGB = hash<int>{}(f.idx());
					auto R = (edgeColorRGB & 0xFF0000) >> 16;
					auto G = (edgeColorRGB & 0x00FF00) >> 8;
					auto B = (edgeColorRGB & 0x0000FF);
					auto edgeColor = vec3(R / 255.f, G / 255.f, B / 255.f);

					Vertex v1;
					v1.Position = mesh.point(heh.from()) + offset_vector;
					v1.Color = edgeColor;
					vertices.push_back(v1);

					Vertex v2;
					v2.Position = mesh.point(heh.to()) + offset_vector;
					v2.Color = edgeColor;
					vertices.push_back(v2);
				}

				// Closest neighbour relationship
				{
					const auto closestNeighbour = closestNeighbours(heh);
					if (closestNeighbour.is_valid())
					{
						const bool is_bridge = closestNeighbours.IsBridge(heh);

						Vertex v1;
						v1.Position = mesh.point(heh.to()) + offset_vector;
						v1.Color = is_bridge ? vec3(1.0f, 223.0f / 255.0f, 0.0f) : vec3(1.0f, 140.0f / 255.0f, 0.0f);
						vertices.push_back(v1);

						Vertex v2;
						v2.Position = mesh.point(closestNeighbour) + offset_vector;
						v2.Color = is_bridge ? vec3(1.0f, 223.0f / 255.0f, 0.0f) : vec3(1.0f, 1.0f, 0.0f);
						vertices.push_back(v2);
					}
				}

				heh = heh.next();
			} while (heh != heh_start);
		}

	return vertices;
}

const mat4& Mesh::GetWorldMat()
{
	return worldMatrix;
}

void Mesh::RotateInCameraSpace(int relx, int rely, const Camera& camera)
{
	// Forgatás kamera koordinátarendszerben
	// https://nicolbolas.github.io/oldtut/Positioning/Tut08%20Camera%20Relative%20Orientation.html
	auto offset = quat(vec3(
		radians((float)rely),
		radians((float)relx),
		0.0f
	));

	
	const vec3& camPos = camera.GetEye();
	const mat4& camMat = camera.GetViewMatrix();

	const fquat viewQuat = quat_cast(camMat);
	const fquat invViewQuat = conjugate(viewQuat);

	const fquat worldQuat = (invViewQuat * offset * viewQuat);
	orientation = worldQuat * orientation;

	UpdateWorldMat();
}

vec3 Mesh::GetColorFromFaceIndex(int idx) const
{
	auto color = (idx * 3) / 3;
	return vec3(1.f - (color & 0xFF) / 255.f, 1.f - ((color >> 8) & 0xFF) / 255.f, 1.f - ((color >> 16) & 0xFF) / 255.f);
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