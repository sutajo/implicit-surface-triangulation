#include "ClosestNeighbours.hpp"
#include "Plane.hpp"
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/type_ptr.hpp>

Implicit::Tessellation::ClosestNeighbours::ClosestNeighbours(GlmPolyMesh& mesh, Object& object) : mesh(mesh), object(object)
{
	gapPoints.request_vertex_status();
}

OpenMesh::VertexHandle Implicit::Tessellation::ClosestNeighbours::AddVertex(OpenMesh::VertexHandle vertex, bool rebuildKdTree)
{
	auto gapVertex = gapPoints.add_vertex(mesh.point(vertex));
	gapPoints.data(gapVertex).connectedVertex = vertex;
	mesh.data(vertex).connectedVertex = gapVertex;
	if (rebuildKdTree)
		gapKdTree.buildIndex();

	return gapVertex;
}

void Implicit::Tessellation::ClosestNeighbours::RemoveVertex(OpenMesh::VertexHandle vertex)
{
	auto gapVertex = mesh.data(vertex).connectedVertex;
	gapPoints.delete_vertex(gapVertex);
}

void Implicit::Tessellation::ClosestNeighbours::RebuildIndex()
{
	gapKdTree.buildIndex();
}

OpenMesh::VertexHandle Implicit::Tessellation::ClosestNeighbours::ComputeClosestNeighbour(OpenMesh::SmartHalfedgeHandle heh)
{
	double r = mesh.calc_edge_length(heh);
	std::vector<std::pair<unsigned int, double>> matches;
	nanoflann::SearchParams searchParams;
	searchParams.sorted = true;
	OpenMesh::VertexHandle closestNeighbour;
	do
	{
		r *= 2.;
		matches.clear();
		gapKdTree.radiusSearch(glm::value_ptr(mesh.point(heh.to())), r, matches, searchParams);

		for (auto& match : matches)
		{
			OpenMesh::VertexHandle gapVertex(match.first);
			if (!gapPoints.status(gapVertex).deleted())
			{
				auto meshVertex = OpenMesh::make_smart(gapPoints.data(gapVertex).connectedVertex, &mesh);
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

void Implicit::Tessellation::ClosestNeighbours::UpdateClosestNeighbour(OpenMesh::SmartHalfedgeHandle heh)
{
	mesh.data(heh.face()).closestNeighbours[heh.to()] = ComputeClosestNeighbour(heh);
}

void Implicit::Tessellation::ClosestNeighbours::UpdateClosestNeighbours(OpenMesh::SmartFaceHandle face)
{
	auto& map = mesh.data(face).closestNeighbours;
	map.clear();
	for (auto heh : face.halfedges())
		map[heh.to()] = ComputeClosestNeighbour(heh);
}

void Implicit::Tessellation::ClosestNeighbours::MoveFaceNeighbours(OpenMesh::FaceHandle from, OpenMesh::FaceHandle to)
{
	auto& moved_to = mesh.data(to).closestNeighbours;
	auto& moved_from = mesh.data(from).closestNeighbours;
	moved_to = std::move(moved_from);
	moved_from = std::unordered_map<OpenMesh::VertexHandle, OpenMesh::VertexHandle>();
}

bool Implicit::Tessellation::ClosestNeighbours::IsConvex(OpenMesh::SmartHalfedgeHandle toHalfedge) const
{
	const auto pointNormal = object.Normal(mesh.point(toHalfedge.to()));
	const auto e1 = mesh.calc_edge_vector(toHalfedge);
	const auto e2 = mesh.calc_edge_vector(toHalfedge.next());
	const double sectorAngle = glm::orientedAngle(e1, e2, pointNormal);
	const bool isConvex = 0.0 <= sectorAngle && sectorAngle <= glm::radians(180.0);
	return isConvex;
}

bool Implicit::Tessellation::ClosestNeighbours::IsNeighbour(OpenMesh::SmartHalfedgeHandle toHalfedge, OpenMesh::VertexHandle vertex) const
{
	const auto planePoint = mesh.point(toHalfedge.to());
	const auto pointNormal = object.Normal(planePoint);
	const auto e1 = mesh.calc_edge_vector(toHalfedge);
	const auto e2 = mesh.calc_edge_vector(toHalfedge.next());
	const double sectorAngle = glm::orientedAngle(e1, e2, pointNormal);
	const bool isConvex = 0.0 <= sectorAngle && sectorAngle <= glm::radians(180.0);
	const Plane pv1{ planePoint, pointNormal, e1 };
	const Plane pv2{ planePoint, pointNormal, e2 };
	const bool isAbovePv1 = pv1.IsAbove(mesh.point(vertex), glm::length(e1) / 10.0);
	const bool isAbovePv2 = pv2.IsAbove(mesh.point(vertex), glm::length(e2) / 10.0);
	if (isConvex) {
		return isAbovePv1 && isAbovePv2;
	}
	else {
		return isAbovePv1 || isAbovePv2;
	}
}

bool Implicit::Tessellation::ClosestNeighbours::IsBridge(OpenMesh::SmartHalfedgeHandle toHalfedge) const
{
	auto closestNeigbour = (*this)(toHalfedge);
	return (*this)(closestNeigbour, toHalfedge.face()) == toHalfedge.to();
}

bool Implicit::Tessellation::ClosestNeighbours::IsBridge(OpenMesh::SmartHalfedgeHandle toHalfedge, OpenMesh::FaceHandle face) const
{
	auto closestNeigbour = (*this)(toHalfedge.to(), face);
	return (*this)(closestNeigbour, face) == toHalfedge.to();
}

OpenMesh::VertexHandle Implicit::Tessellation::ClosestNeighbours::operator()(OpenMesh::HalfedgeHandle toHalfedge) const
{
	return (*this)(mesh.to_vertex_handle(toHalfedge), mesh.face_handle(toHalfedge));
}

OpenMesh::VertexHandle Implicit::Tessellation::ClosestNeighbours::operator()(OpenMesh::VertexHandle vertex, OpenMesh::FaceHandle face) const
{
	const auto& map = mesh.data(face).closestNeighbours;
	const auto it = map.find(vertex);
	return it != map.end() ? it->second : OpenMesh::VertexHandle();
}