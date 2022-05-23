#include "Plane.hpp"
#include "FillingPhase.hpp"
#include <glm/gtx/vector_angle.hpp>

Implicit::Tessellation::FillingPhase::FillingPhase(GlmPolyMesh& mesh, Object& object) : 
	Phase(mesh, mesh),
	mesh(mesh),
	object(object),
	closestNeighbours(mesh, object)
{
	mesh.request_face_status();
}

void Implicit::Tessellation::FillingPhase::Start()
{
	OpenMesh::SmartHalfedgeHandle heh_start = FindBoundaryHalfEdge(mesh);
	std::vector<OpenMesh::VertexHandle> gapVertices;
	auto heh = heh_start;
	do
	{
		auto heh_to = heh.to();
		closestNeighbours.AddVertex(heh_to);
		gapVertices.push_back(heh_to);
		heh = heh.next();
	} while (heh != heh_start);

	closestNeighbours.RebuildIndex();

	auto initial_gap = mesh.add_face(gapVertices);
	if (!initial_gap.is_valid())
		throw std::runtime_error("Can't create initial gap");

	closestNeighbours.UpdateClosestNeighbours(initial_gap);

	gaps.push_back(initial_gap);
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
		EarFilling(gap, false) ||
		ConvexPolygonFilling(gap) ||
		EarFilling(gap, true) ||
		ConcaveVertexBisection(gap);
	}
}

bool Implicit::Tessellation::FillingPhase::Completed() const
{
	return gaps.empty();
}

const Implicit::Tessellation::ClosestNeighbours& Implicit::Tessellation::FillingPhase::GetClosestNeighbours() const
{
	return closestNeighbours;
}

bool Implicit::Tessellation::FillingPhase::SmallPolygonFilling(OpenMesh::FaceHandle gap)
{
	const auto valence = mesh.valence(gap);
	if (valence == 3)
		mesh.data(gap).faceCreationMethod = FaceCreationMethod::SmallPolygonFilling;
	else if(mesh.valence(gap) != 4)
		return false;

	//std::cout << "SmallPolygonFilling: " << gap.idx() << std::endl;

	bool allVerticesConvex = true;
	for (auto heh : mesh.fh_range(gap))
	{
		if (!closestNeighbours.IsConvex(heh))
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

	auto heh_start = OpenMesh::make_smart(mesh.halfedge_handle(gap), &mesh);
	auto heh = heh_start;
	do
	{
		auto closestNeighbour = OpenMesh::make_smart(closestNeighbours(heh), &mesh);
		const bool separated = heh.next().next().to() != closestNeighbour && heh.prev().from() != closestNeighbour;
		if (closestNeighbours.IsBridge(heh) && separated)
		{
			//std::cout << "SubdivisionOnBridges: " << gap.idx() << std::endl;
			mesh.data(heh.opp().face()).faceCreationMethod = FaceCreationMethod::Seed;

			auto closestNeighbour_heh = closestNeighbour.outgoing_halfedges().first([&](OpenMesh::HalfedgeHandle h) { return mesh.face_handle(h) == heh.face(); });
			assert(closestNeighbour_heh.is_valid());

			auto newface_halfege = OpenMesh::make_smart(mesh.insert_edge(heh, closestNeighbour_heh), &mesh);
			closestNeighbours.UpdateClosestNeighbours(newface_halfege.face());
			closestNeighbours.UpdateClosestNeighbours(newface_halfege.opp().face());

			heh_start = heh.face() != newface_halfege.face() ? newface_halfege : newface_halfege.opp();
			heh = heh_start;
			gaps.push_back(newface_halfege.face());

			meshChanged = true;
			break;
		}

		heh = heh.next();
	} while (heh != heh_start);

	if (meshChanged)
	{
		gaps.push_back(heh.face());
	}

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

		if (closestNeighbours(v2, heh.face()) == v4 && closestNeighbours(v3, heh.face()) == v1)
		{
			bool closestNeighbourOutsideGap = false;
			// Check that neither v2 nor v3 is the closest neigbour of any vertex not in the current gap
			auto heh_nn = heh_next_next;
			while (heh_nn.next() != heh.prev())
			{
				auto closestNeighbour = closestNeighbours(heh_nn.next());
				closestNeighbourOutsideGap = closestNeighbour == v2 || closestNeighbour == v3;
				if (closestNeighbourOutsideGap)
					break;

				heh_nn = heh_nn.next();
			}

			if (!closestNeighbourOutsideGap)
			{
				//std::cout << "XFilling: Face: " << heh.face().idx() << std::endl;
				//std::cout << "XFilling: v1: " << v1.idx() << " v2:" << v2.idx() << " v3:" << v3.idx() << " v4:" << v4.idx() << std::endl;

				closestNeighbours.RemoveVertex(v2);
				closestNeighbours.RemoveVertex(v3);

				auto d1 = glm::distance(mesh.point(v1), mesh.point(v2));
				auto d2 = glm::distance(mesh.point(v2), mesh.point(v3));
				auto d3 = glm::distance(mesh.point(v3), mesh.point(v4));
				auto d = glm::distance(mesh.point(v1), mesh.point(v4));

				const auto newface_halfege = OpenMesh::make_smart(mesh.insert_edge(heh.prev(), heh_next_next.next()), &mesh);
				const auto newface_halfege_opp = newface_halfege.opp();
				const auto original_gap = newface_halfege.face();
				const auto xfilled_face = newface_halfege_opp.face();

				//std::cout << "XFilling: Big face: " << original_gap.idx() << std::endl;
				//std::cout << "XFilling: Small face: " << xfilled_face.idx() << std::endl;

				mesh.data(xfilled_face).faceCreationMethod = FaceCreationMethod::XFilling;
				meshChanged = true;

				heh_start = newface_halfege.prev();
				heh = heh_start;

				closestNeighbours.MoveFaceNeighbours(xfilled_face, original_gap);

				if (d > 1.5 * std::max({ d1, d2, d3 }))
				{
					auto midpoint_vertex = (mesh.point(v1) + mesh.point(v4)) / 2.;
					midpoint_vertex = object.Project(midpoint_vertex);
					auto mv_handle = mesh.add_vertex(midpoint_vertex);

					mesh.split_edge(newface_halfege.edge(), mv_handle);

					auto to_mv = newface_halfege_opp;
					auto to_mv_prev = to_mv.prev();

					assert(to_mv.to() == mv_handle);
					assert(to_mv_prev.from() == v3);

					// Add 3 new faces
					to_mv = OpenMesh::make_smart(mesh.insert_edge(to_mv, to_mv_prev), &mesh).opp();
					to_mv_prev = to_mv.prev();
					mesh.data(to_mv.opp().face()).faceCreationMethod = FaceCreationMethod::XFilling;

					assert(to_mv.is_valid());
					assert(to_mv.to() == mv_handle);
					assert(to_mv_prev.from() == v2);

					to_mv = OpenMesh::make_smart(mesh.insert_edge(to_mv, to_mv_prev), &mesh);
					assert(to_mv.is_valid());

					mesh.data(to_mv.face()).faceCreationMethod = FaceCreationMethod::XFilling;

					// Update closest neighbours for big face
				    closestNeighbours.AddVertex(mv_handle, true);
					closestNeighbours.UpdateClosestNeighbours(original_gap);
				}
				else
				{
					closestNeighbours.UpdateClosestNeighbour(newface_halfege.prev());
					closestNeighbours.UpdateClosestNeighbour(newface_halfege);

					gaps.push_back(xfilled_face);
				}
			}
		}

		heh = heh.next();
	} while (heh != heh_start);

	if (meshChanged)
	{
		gaps.push_back(heh.face());
	}

	return meshChanged;
}

bool Implicit::Tessellation::FillingPhase::EarFilling(OpenMesh::FaceHandle gap, bool relaxed)
{
	auto sgap = OpenMesh::make_smart(gap, &mesh);
	bool meshChanged = false;

	OpenMesh::SmartHalfedgeHandle heh_start = OpenMesh::make_smart(mesh.halfedge_handle(gap), &mesh);
	auto heh = heh_start;
	do
	{
		auto v1 = heh.from();
		auto v2 = heh.to();
		auto v3 = heh.next().to();

		const bool cn_v1_v3 = closestNeighbours(v1, heh.face()) == v3;
		const bool cn_v3_v1 = closestNeighbours(v3, heh.face()) == v1;

		const bool earNeedsFilling = relaxed ? (cn_v1_v3 || cn_v3_v1) : (cn_v1_v3 && cn_v3_v1);
		if (earNeedsFilling)
		{	
			const auto heh_next_next = heh.next().next();
			bool closestNeighbourOutsideGap = false;
			// Check that neither v2 nor v3 is the closest neigbour of any vertex not in the current gap
			auto heh_nn = heh_next_next;
			while (heh_nn != heh.prev())
			{
				auto closestNeighbour = closestNeighbours(heh_nn);
				closestNeighbourOutsideGap = closestNeighbour == v2;
				if (closestNeighbourOutsideGap)
					break;

				heh_nn = heh_nn.next();
			}

			if (!closestNeighbourOutsideGap)
			{
				//std::cout << "EarFilling: Face: " << heh.face().idx() << std::endl;
				//std::cout << "EarFilling: v1: " << v1.idx() << " v2:" << v2.idx() << " v3:" << v3.idx() << std::endl;

				closestNeighbours.RemoveVertex(v2);

				auto d1 = glm::distance(mesh.point(v1), mesh.point(v2));
				auto d2 = glm::distance(mesh.point(v2), mesh.point(v3));
				auto d = glm::distance(mesh.point(v1), mesh.point(v3));

				const auto newface_halfege = OpenMesh::make_smart(mesh.insert_edge(heh.prev(), heh_next_next), &mesh);
				const auto original_gap = newface_halfege.face();
				const auto earfilled_face = newface_halfege.opp().face();

				mesh.data(earfilled_face).faceCreationMethod = relaxed ? FaceCreationMethod::RelaxedEarFilling : FaceCreationMethod::EarFilling;
				meshChanged = true;

				heh_start = newface_halfege.prev();
				heh = heh_start;

				closestNeighbours.MoveFaceNeighbours(earfilled_face, original_gap);

				if (d > 1.5 * std::max({ d1, d2 }))
				{
					const auto midpoint = (mesh.point(v1) + mesh.point(v3)) / 2.;
					const auto mv_handle = mesh.add_vertex(midpoint);
					mesh.split_edge(newface_halfege.edge(), mv_handle);

					// Update closest neighbours for big face
					closestNeighbours.AddVertex(mv_handle, true);
					closestNeighbours.UpdateClosestNeighbours(original_gap);

					gaps.push_back(earfilled_face);
				}
				else
				{ 
					closestNeighbours.UpdateClosestNeighbour(newface_halfege.prev());
					closestNeighbours.UpdateClosestNeighbour(newface_halfege);
				}
			}
		}

		heh = heh.next();
	} while (heh != heh_start);

	if (meshChanged)
	{
		gaps.push_back(heh.face());
	}

	return meshChanged;
}

bool Implicit::Tessellation::FillingPhase::ConvexPolygonFilling(OpenMesh::FaceHandle gap)
{
	auto sgap = OpenMesh::make_smart(gap, &mesh);

	for (auto heh : sgap.halfedges())
		if (!closestNeighbours.IsConvex(heh))
			return false;

	// All vertices are convex, go!
	//std::cout << "ConvexPolygonFilling: " << gap.idx() << std::endl;

	const auto midpoint = mesh.calc_centroid(gap);
	mesh.split(gap, mesh.add_vertex(midpoint));

	return true;
}

bool Implicit::Tessellation::FillingPhase::ConcaveVertexBisection(OpenMesh::FaceHandle gap)
{
	auto getInteriorAngle = [&](const auto &halfedge, const auto& e2) -> double
	{
		const auto pointNormal = object.Normal(mesh.point(halfedge.to()));
		const auto e1 = mesh.calc_edge_vector(halfedge);
		auto angle = acos(glm::dot(glm::normalize(e1), glm::normalize(e2)));
		const auto cross = glm::cross(e1, e2);
		if (glm::dot(pointNormal, cross) < 0) { // Or > 0
			angle += M_PI;
		}
		return angle;
	};

	std::cout << "ConcaveVertexBisection: " << gap.idx() << std::endl;

	auto sgap = OpenMesh::make_smart(gap, &mesh);

	auto liah = sgap.halfedge(); // Largest interior angle halfedge
	double lia = getInteriorAngle(liah, mesh.calc_edge_vector(liah.next()));
	
	for (auto heh : sgap.halfedges())
	{
		const double interiorAngle = getInteriorAngle(heh, mesh.calc_edge_vector(heh.next()));
		if (interiorAngle > lia)
		{
			lia = interiorAngle;
			liah = heh;
		}
	}

	std::cout << "Interior angle: " << glm::degrees(lia) << std::endl;

	const auto liahP = mesh.point(liah.to());
	auto liahN = object.Normal(liahP);
	const auto e1 = mesh.calc_edge_vector(liah);

	double nearestBisection = std::numeric_limits<double>::max();
	OpenMesh::HalfedgeHandle nearestBisector;
	auto liah_prev = liah.prev();
	for (auto heh : sgap.halfedges())
	{
		if (heh == liah.next() || heh == liah_prev ||  heh == liah)
			continue;

		const auto e2 = mesh.point(heh.to()) - liahP;
		double sectorAngle = getInteriorAngle(heh, e2);
		
		const double bisectionDistance = std::abs(sectorAngle - lia / 2.);
		if (!nearestBisector.is_valid() || bisectionDistance < nearestBisection)
		{
			nearestBisection = bisectionDistance;
			nearestBisector = heh;
		}
	}
	if (!nearestBisector.is_valid())
		throw std::runtime_error("Impossible");

	std::cout << "Closest bisector: " << glm::degrees(nearestBisection) << std::endl;

	const auto newface_halfege = OpenMesh::make_smart(mesh.insert_edge(liah, mesh.next_halfedge_handle(nearestBisector)), &mesh);

	const auto newface_halfege_opp = newface_halfege.opp();
	const auto original_gap = newface_halfege.face();
	const auto new_face = newface_halfege_opp.face();
	
	closestNeighbours.UpdateClosestNeighbours(original_gap);
	closestNeighbours.UpdateClosestNeighbours(new_face);

	gaps.push_back(original_gap);
	gaps.push_back(new_face);

	//mesh.data(mesh.opposite_face_handle(nearestBisector)).faceCreationMethod = FaceCreationMethod::EarCutting;
 	//mesh.data(liah.opp().face()).faceCreationMethod = FaceCreationMethod::EarCutting;

	return true;
}

