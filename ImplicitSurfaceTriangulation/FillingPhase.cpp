#include "Plane.hpp"
#include "FillingPhase.hpp"
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/type_ptr.hpp>

Implicit::Tessellation::FillingPhase::FillingPhase(GlmPolyMesh& mesh, Object& object) : Phase(mesh, mesh), mesh(mesh), object(object)
{
	gapPoints.request_vertex_status();
	mesh.request_face_status();
}

void Implicit::Tessellation::FillingPhase::Start()
{
	computeClosestNeighbours();
}

void Implicit::Tessellation::FillingPhase::RunIterations(int iterations)
{
	for (int i = 0; i < iterations && !Completed(); ++i)
	{
		auto gap = gaps.front();
		gaps.pop_front();

		SmallPolygonFilling(gap) ||
		SubdivisionOnBridges(gap) ||
		XFilling(gap) ||
		EarFilling(gap) ||
		RelaxedEarFilling(gap) ||
		ConcaveVertexBisection(gap);
	}
}

bool Implicit::Tessellation::FillingPhase::Completed() const
{
	return gaps.empty();
}

OpenMesh::VertexHandle Implicit::Tessellation::FillingPhase::addGapVertex(OpenMesh::VertexHandle vertex, bool rebuildKdTree)
{
	auto gapVertex = gapPoints.add_vertex(mesh.point(vertex));
	gapPoints.data(gapVertex).connectedVertex = vertex;
	mesh.data(vertex).connectedVertex = gapVertex;
	if (rebuildKdTree)
		gapKdTree.buildIndex();

	return gapVertex;
}

void Implicit::Tessellation::FillingPhase::removeGapVertex(OpenMesh::VertexHandle vertex)
{
	auto gapVertex = mesh.data(vertex).connectedVertex;
	gapPoints.delete_vertex(gapVertex);
}

OpenMesh::VertexHandle Implicit::Tessellation::FillingPhase::computeClosestNeighbour(OpenMesh::SmartHalfedgeHandle heh)
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
				auto meshVertex = gapPoints.data(gapVertex).connectedVertex;
				if (isNeighbour(heh, meshVertex))
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

void Implicit::Tessellation::FillingPhase::computeClosestNeighbours()
{
	OpenMesh::SmartHalfedgeHandle heh_start = FindBoundaryHalfEdge(mesh);

	std::vector<OpenMesh::VertexHandle> gapVertices;

	auto heh = heh_start;
	do
	{
		auto heh_to = heh.to();
		addGapVertex(heh_to);
		gapVertices.push_back(heh_to);
		heh = heh.next();
	} while (heh != heh_start);

	gapKdTree.buildIndex();

	heh = heh_start;
	do
	{
		mesh.data(heh.to()).closestNeighbour = computeClosestNeighbour(heh);
		heh = heh.next();
	} while (heh != heh_start);

	auto initial_gap = mesh.add_face(gapVertices);
	if (!initial_gap.is_valid())
		throw std::runtime_error("Can't create initial gap");

	gaps.push_back(initial_gap);
}

bool Implicit::Tessellation::FillingPhase::isConvex(OpenMesh::SmartHalfedgeHandle toHalfedge)
{
	const double sectorAngle = mesh.calc_sector_angle(toHalfedge);
	const bool isConvex = 0.0 <= sectorAngle && sectorAngle <= glm::radians(180.0);
	return isConvex;
}

bool Implicit::Tessellation::FillingPhase::isNeighbour(OpenMesh::SmartHalfedgeHandle toHalfedge, OpenMesh::VertexHandle vertex)
{
	const bool isConvex = this->isConvex(toHalfedge);
	const auto planePoint = mesh.point(toHalfedge.to());
	const auto pointNormal = object.Normal(planePoint);
	const Plane pv1{ planePoint, pointNormal, mesh.calc_edge_vector(toHalfedge) };
	const Plane pv2{ planePoint, pointNormal, mesh.calc_edge_vector(toHalfedge.next()) };
	const bool isAbovePv1 = pv1.IsAbove(mesh.point(vertex), mesh.calc_edge_length(toHalfedge) / 10.0);
	const bool isAbovePv2 = pv2.IsAbove(mesh.point(vertex), mesh.calc_edge_length(toHalfedge.next()) / 10.0);
	if (isConvex) {
		return isAbovePv1 && isAbovePv2;
	}
	else {
		return isAbovePv1 || isAbovePv2;
	}
}

bool Implicit::Tessellation::FillingPhase::isBridge(OpenMesh::VertexHandle vertex)
{
	auto closestNeigbour = mesh.data(vertex).closestNeighbour;
	return mesh.data(closestNeigbour).closestNeighbour == vertex;
}

bool Implicit::Tessellation::FillingPhase::SmallPolygonFilling(OpenMesh::FaceHandle gap)
{
	if (mesh.valence(gap) != 4)
		return false;

	bool allVerticesConvex = true;
	for (auto heh : mesh.fh_range(gap))
	{
		if (!isConvex(heh))
		{
			allVerticesConvex = false;
			break;
		}
	}

	if (allVerticesConvex)
	{
		auto heh = OpenMesh::make_smart(mesh.halfedge_handle(gap), &mesh);
		auto p1 = mesh.point(heh.from());
		auto p2 = mesh.point(heh.to());
		auto heh_nn = heh.next().next();
		auto p3 = mesh.point(heh_nn.from());
		auto p4 = mesh.point(heh_nn.to());

		auto side1 = glm::distance(p1, p2);
		auto side2 = glm::distance(p2, p3);
		auto side3 = glm::distance(p3, p4);
		auto side4 = glm::distance(p4, p1);

		const double shortestSide = std::min({ side1, side2, side3, side4 });
		const double longestSide = std::max({ side1, side2, side3, side4 });

		const double diagonal1 = glm::distance(p1, p3);
		const double diagonal2 = glm::distance(p2, p4);

		const double quality1 = std::min({ shortestSide, diagonal1 }) / std::max({ longestSide, diagonal1 });
		const double quality2 = std::min({ shortestSide, diagonal2 }) / std::max({ longestSide, diagonal2 });

		OpenMesh::HalfedgeHandle new_heh;
		if (quality1 > quality2)
		{
			new_heh = mesh.insert_edge(heh.prev(), heh_nn);
		}
		else
		{
			new_heh = mesh.insert_edge(heh, heh_nn.next());
		}

		auto new_face = mesh.face_handle(new_heh);
		auto new_face_opp = mesh.opposite_face_handle(new_heh);
		auto creationMethod = mesh.data(gap).faceCreationMethod == FaceCreationMethod::XFilling ? FaceCreationMethod::XFilling : FaceCreationMethod::SmallPolygonFilling;
		mesh.data(new_face).faceCreationMethod = creationMethod;
		mesh.data(new_face_opp).faceCreationMethod = creationMethod;
	}
	else
	{
		mesh.triangulate(gap);
	}

	return true;
}

bool Implicit::Tessellation::FillingPhase::SubdivisionOnBridges(OpenMesh::FaceHandle gap)
{
	bool meshChanged = false;
	/*
	auto heh_start = OpenMesh::make_smart(mesh.halfedge_handle(gap), &mesh);
	auto heh = heh_start;
	do
	{
		auto v = heh.to();
		auto closestNeighbour = OpenMesh::make_smart(mesh.data(v).closestNeighbour, &mesh);
		if (isBridge(v) && v.idx() < closestNeighbour.idx())
		{
			auto closestNeighbour_heh = closestNeighbour.outgoing_halfedges().first([&](OpenMesh::HalfedgeHandle h) { return mesh.face_handle(h) == gap; });
			if (!closestNeighbour_heh.is_valid())
				throw std::runtime_error("Halfege handle not found");

			auto newface_halfege = OpenMesh::make_smart(mesh.insert_edge(heh, closestNeighbour_heh), &mesh);
			auto new_face = mesh.face_handle(newface_halfege);
			auto new_face_opp = mesh.opposite_face_handle(newface_halfege);
			gaps.push_back(new_face);
			meshChanged = true;
			break;
		}

		heh = heh.next();
	} while (heh != heh_start);
	*/
	return meshChanged;
}

bool Implicit::Tessellation::FillingPhase::XFilling(OpenMesh::FaceHandle gap)
{
	auto sgap = OpenMesh::make_smart(gap, &mesh);
	if (sgap.valence() < 4)
		return false;

	bool meshChanged = false;

	OpenMesh::SmartHalfedgeHandle heh_start = OpenMesh::make_smart(mesh.halfedge_handle(gap), &mesh);
	auto heh = heh_start;
	do
	{
		auto v1 = heh.from();
		auto v2 = heh.to();
		const auto heh_next_next = heh.next().next();
		auto v3 = heh_next_next.from();
		auto v4 = heh_next_next.to();

		if (mesh.data(v2).closestNeighbour == v4 && mesh.data(v3).closestNeighbour == v1)
		{
			bool closestNeighbourOutsideGap = false;
			// Check that neither v2 nor v3 is the closest neigbour of any vertex not in the current gap
			auto heh_nn = heh_next_next;
			while (heh_nn.next() != heh.prev())
			{
				auto closestNeighbour = mesh.data(heh_next_next.to()).closestNeighbour;
				closestNeighbourOutsideGap = closestNeighbour == v2 || closestNeighbour == v3;
				if (closestNeighbourOutsideGap)
					break;

				heh_nn = heh_nn.next();
			}

			if (!closestNeighbourOutsideGap)
			{
				removeGapVertex(v2);
				removeGapVertex(v3);

				auto d1 = glm::distance(mesh.point(v1), mesh.point(v2));
				auto d2 = glm::distance(mesh.point(v2), mesh.point(v3));
				auto d3 = glm::distance(mesh.point(v3), mesh.point(v4));
				auto d = glm::distance(mesh.point(v1), mesh.point(v4));

				if (d > 1.5 * std::max({ d1, d2, d3 }))
				{
					auto midpoint_vertex = (mesh.point(v1) + mesh.point(v4)) / 2.;
					midpoint_vertex = object.Project(midpoint_vertex);
					auto mv_handle = mesh.add_vertex(midpoint_vertex);
					if (!mesh.add_face(v1, v2, mv_handle).is_valid() ||
						!mesh.add_face(v2, v3, mv_handle).is_valid() ||
						!mesh.add_face(v3, v4, mv_handle).is_valid())
						throw std::runtime_error("Failed to create new faces");
				}
				else
				{
					auto newface_halfege = OpenMesh::make_smart(mesh.insert_edge(heh.prev(), heh_next_next.next()), &mesh);
					auto new_face = mesh.face_handle(newface_halfege);
					auto new_face_opp = mesh.opposite_face_handle(newface_halfege);

					if (mesh.data(v1).closestNeighbour == v4)
						mesh.data(v1).closestNeighbour = computeClosestNeighbour(newface_halfege.prev());
					if (mesh.data(v4).closestNeighbour == v1)
						mesh.data(v4).closestNeighbour = computeClosestNeighbour(newface_halfege);

					mesh.data(new_face_opp).faceCreationMethod = FaceCreationMethod::XFilling;
					gaps.push_back(new_face_opp);
					gaps.push_back(new_face);
					meshChanged = true;
					break;
				}
			}
		}

		heh = heh.next();
	} while (heh != heh_start);

	return meshChanged;
}

bool Implicit::Tessellation::FillingPhase::EarFilling(OpenMesh::FaceHandle gap)
{
	return false;
}

bool Implicit::Tessellation::FillingPhase::ConvexPolygonFilling(OpenMesh::FaceHandle gap)
{
	return false;
}

bool Implicit::Tessellation::FillingPhase::RelaxedEarFilling(OpenMesh::FaceHandle gap)
{
	return false;
}

bool Implicit::Tessellation::FillingPhase::ConcaveVertexBisection(OpenMesh::FaceHandle gap)
{
	return false;
}

