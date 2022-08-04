#include "ClosestNeighbours.hpp"
#include "Plane.hpp"
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace Implicit::Tessellation;
using namespace OpenMesh;
using namespace std;
using namespace nanoflann;

ClosestNeighbours::ClosestNeighbours(GlmPolyMesh& mesh, Object& object) : mesh(mesh), object(object)
{
	gapPoints.request_vertex_status();
}

VertexHandle ClosestNeighbours::AddVertex(VertexHandle vertex, bool rebuildKdTree)
{
	auto gapVertex = gapPoints.add_vertex(mesh.point(vertex));
	gapPoints.data(gapVertex).connectedVertex = vertex;
	mesh.data(vertex).connectedVertex = gapVertex;
	if (rebuildKdTree)
		gapKdTree.buildIndex();

	return gapVertex;
}

void ClosestNeighbours::RemoveVertex(VertexHandle vertex)
{
	auto gapVertex = mesh.data(vertex).connectedVertex;
	gapPoints.delete_vertex(gapVertex);
}

void ClosestNeighbours::RebuildIndex()
{
	gapKdTree.buildIndex();
}

VertexHandle ClosestNeighbours::ComputeClosestNeighbour(SmartHalfedgeHandle heh)
{
	double r = mesh.calc_edge_length(heh);
	vector<pair<unsigned int, double>> matches;
	SearchParams searchParams;
	searchParams.sorted = true;
	VertexHandle closestNeighbour;
	do
	{
		r *= 2.;
		matches.clear();
		gapKdTree.radiusSearch(glm::value_ptr(mesh.point(heh.to())), r, matches, searchParams);

		for (auto& match : matches)
		{
			VertexHandle gapVertex(match.first);
			if (!gapPoints.status(gapVertex).deleted())
			{
				auto meshVertex = make_smart(gapPoints.data(gapVertex).connectedVertex, &mesh);
				const bool isInSameFace = meshVertex.faces().any_of([&](auto face) { return face == heh.face(); });
				if (isInSameFace && IsNeighbour(heh, meshVertex))
				{
					closestNeighbour = meshVertex;
					break;
				}
			}
		}

		if (matches.size() == gapPoints.n_vertices())
		{
			break;
		}
	} while (!closestNeighbour.is_valid());
	return closestNeighbour;
}

void ClosestNeighbours::UpdateClosestNeighbour(SmartHalfedgeHandle heh)
{
	mesh.data(heh.face()).closestNeighbours[heh.to()] = ComputeClosestNeighbour(heh);
}

void ClosestNeighbours::UpdateClosestNeighbours(SmartFaceHandle face)
{
	auto& map = mesh.data(face).closestNeighbours;
	map.clear();
	for (auto heh : face.halfedges())
		map[heh.to()] = ComputeClosestNeighbour(heh);
}

void ClosestNeighbours::MoveFaceNeighbours(FaceHandle from, FaceHandle to)
{
	auto& moved_to = mesh.data(to).closestNeighbours;
	auto& moved_from = mesh.data(from).closestNeighbours;
	moved_to = move(moved_from);
	moved_from = unordered_map<VertexHandle, VertexHandle>();
}

bool ClosestNeighbours::IsConvex(SmartHalfedgeHandle toHalfedge) const
{
	const auto pointNormal = object.Normal(mesh.point(toHalfedge.to()));
	const auto e1 = mesh.calc_edge_vector(toHalfedge);
	const auto e2 = mesh.calc_edge_vector(toHalfedge.next());
	const double sectorAngle = glm::orientedAngle(e1, e2, pointNormal);
	const bool isConvex = 0.0 <= sectorAngle && sectorAngle <= glm::radians(180.0);
	return isConvex;
}

bool ClosestNeighbours::IsNeighbour(SmartHalfedgeHandle toHalfedge, VertexHandle vertex) const
{
	const auto planePoint = mesh.point(toHalfedge.to());
	const auto pointNormal = object.Normal(planePoint);
	const auto e1 = mesh.calc_edge_vector(toHalfedge);
	const auto e2 = mesh.calc_edge_vector(toHalfedge.next());
	const double sectorAngle = glm::orientedAngle(e1, e2, pointNormal);
	const bool isConvex = 0.0 <= sectorAngle && sectorAngle <= glm::radians(180.0);
	const Plane pv1{ planePoint, pointNormal, e1 };
	const Plane pv2{ planePoint, pointNormal, e2 };
	const bool isAbovePv1 = pv1.IsAbove(mesh.point(vertex), length(e1) / 10.0);
	const bool isAbovePv2 = pv2.IsAbove(mesh.point(vertex), length(e2) / 10.0);
	if (isConvex) {
		return isAbovePv1 && isAbovePv2;
	}
	else {
		return isAbovePv1 || isAbovePv2;
	}
}

bool ClosestNeighbours::IsBridge(SmartHalfedgeHandle toHalfedge) const
{
	auto closestNeigbour = (*this)(toHalfedge);
	return (*this)(closestNeigbour, toHalfedge.face()) == toHalfedge.to();
}

bool ClosestNeighbours::IsBridge(SmartHalfedgeHandle toHalfedge, FaceHandle face) const
{
	auto closestNeigbour = (*this)(toHalfedge.to(), face);
	return (*this)(closestNeigbour, face) == toHalfedge.to();
}

VertexHandle ClosestNeighbours::operator()(HalfedgeHandle toHalfedge) const
{
	return (*this)(mesh.to_vertex_handle(toHalfedge), mesh.face_handle(toHalfedge));
}

VertexHandle ClosestNeighbours::operator()(VertexHandle vertex, FaceHandle face) const
{
	const auto& map = mesh.data(face).closestNeighbours;
	const auto it = map.find(vertex);
	return it != map.end() ? it->second : VertexHandle();
}