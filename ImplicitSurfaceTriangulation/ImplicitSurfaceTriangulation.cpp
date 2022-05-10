#include "ImplicitSurfaceTriangulation.hpp"
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/type_ptr.hpp>

Implicit::CurvatureTessellator::CurvatureTessellator(Object& object) 
    : object(object), visitor()
{
    mesh.request_face_normals();
    mesh.request_vertex_normals();
    gap.request_vertex_status();
    omerr().rdbuf(std::cerr.rdbuf());
}

Implicit::CurvatureTessellator::CurvatureTessellator(Object& object, Visitor& visitor) 
    : object(object), visitor(visitor)
{
    mesh.request_face_normals();
    mesh.request_vertex_normals();
    gap.request_vertex_status();
    omerr().rdbuf(std::cerr.rdbuf());
}

void Implicit::CurvatureTessellator::SetRho(double rho)
{
    this->rho = rho;
}

void Implicit::CurvatureTessellator::GenerateSeedTriangle()
{
    const glm::dvec3 seedPoint =  object.GetStartVertex();
    const glm::dvec3 normal = object.Normal(seedPoint);

    glm::dvec3 t, b;
    object.getTangentSpace(normal, t, b);

    const double roc = getRoc(seedPoint);
    const double sideLength = roc * rho;

    const glm::dvec3 nextPoint = object.Project(seedPoint + t * sideLength);
    const glm::dvec3 finalPoint = getEtp(seedPoint, nextPoint, b); // object.Project();

    if (!object.GetBoundingBox().contains(nextPoint) ||
        !object.GetBoundingBox().contains(finalPoint))
        throw std::runtime_error("Failed to generate seed triangle: vertices fall out of the bounding box");

    auto newFace = addNewFace(seedPoint, nextPoint, finalPoint);
    mesh.data(newFace).faceCreationMethod = FaceCreationMethod::Seed;

    if (visitor)
        visitor->get().SeedTriangleGenerated(Triangle{seedPoint, nextPoint, finalPoint});
}

void Implicit::CurvatureTessellator::ComputeMesh()
{
    
}

const GlmPolyMesh& Implicit::CurvatureTessellator::GetMesh()
{
    return mesh;
}

const Aabb Implicit::CurvatureTessellator::ComputeBoundingBox()
{
    GlmPolyMeshKdTree::BoundingBox bb;
    kdTree.computeBoundingBox(bb);
    Aabb aabb;
    auto min = glm::dvec3(bb[0].low, bb[1].low, bb[2].low);
    auto max = glm::dvec3(bb[0].high, bb[1].high, bb[2].high);
    aabb.include(min);
    aabb.include(max);
    return aabb;
}

double Implicit::CurvatureTessellator::getRoc(const glm::dvec3& point) const
{
    double k1, k2;
    object.Curvature(point, k1, k2);
    return 1.0f / ((std::max(std::abs(k1), std::abs(k2))));;
}

glm::dvec3 Implicit::CurvatureTessellator::getEtp(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& normalizedTangent) const
{
    return getItp(a, b, normalizedTangent, glm::distance(a, b));
}

glm::dvec3 Implicit::CurvatureTessellator::getItp(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& normalizedTangent, double equalSideLength) const
{
    const glm::dvec3 midPoint = (a + b) / 2.;
    const double midDistance = glm::distance(midPoint, b);
    if (midDistance >= equalSideLength) {
        std::cerr << "equalSideLength is too small\n";
        equalSideLength = midDistance;
    }

    const double height = sqrt(equalSideLength*equalSideLength - midDistance*midDistance);
    return midPoint + height * normalizedTangent;
}

Plane Implicit::CurvatureTessellator::getPlaneFromHalfEdge(const OpenMesh::SmartHalfedgeHandle& heh) const
{
    auto& point = mesh.point(heh.to());
    return Plane(point, object.Normal(point), mesh.calc_edge_vector(heh));
}

OpenMesh::SmartFaceHandle Implicit::CurvatureTessellator::addNewFace(const glm::dvec3& pointA, const glm::dvec3& pointB, const glm::dvec3& pointC)
{
    const double longestSideHere = Triangle{ pointA, pointB, pointC }.GetLongestSide();
    auto faceHandle = mesh.add_face(mesh.add_vertex(pointA), mesh.add_vertex(pointB), mesh.add_vertex(pointC));
    if (faceHandle == GlmTriMesh::FaceHandle()) {
        mesh.clean();
        throw std::runtime_error("Failed to create new face");
    }

    kdTreeIsDirty = true;

    if (longestSideHere > longestSide)
    {
        longestSide = longestSideHere;
        longestSidedFace = faceHandle;
    }

    return faceHandle;
}

OpenMesh::SmartFaceHandle Implicit::CurvatureTessellator::addNewFace(const OpenMesh::VertexHandle& pointA, const OpenMesh::VertexHandle& pointB, const glm::dvec3& pointC, double newTriangleLongestSide)
{
    auto newVertex = mesh.add_vertex(pointC);
    if (newVertex == GlmTriMesh::VertexHandle())
        throw std::runtime_error("Failed to create new vertex");

    auto faceHandle = mesh.add_face(pointA, pointB, newVertex);
    if (faceHandle == GlmTriMesh::FaceHandle()) {
        mesh.delete_vertex(newVertex);
        throw std::runtime_error("Failed to create new face");
    }

    kdTreeIsDirty = true;

    if (newTriangleLongestSide > longestSide)
    {
        longestSide = newTriangleLongestSide;
        longestSidedFace = faceHandle;
    }

    return faceHandle;
}

bool Implicit::CurvatureTessellator::expandEdge(OpenMesh::SmartEdgeHandle edge, OpenMesh::SmartFaceHandle& newFace)
{
    const bool grownAlready = std::exchange(mesh.data(edge).grownAlready, true);
    if (grownAlready)
        return false; // The edge was grown already

    if (!mesh.is_boundary(edge))
        throw std::runtime_error("Only boundary edges can be expanded");

    // Get the face corresponding to the boundary edge
    auto heh = mesh.halfedge_handle(edge, 0);
    if(mesh.is_boundary(heh))
        heh = mesh.halfedge_handle(edge, 1);

    const glm::dvec3 P0 = mesh.point(edge.v0());
    const glm::dvec3 P1 = mesh.point(edge.v1());

    // const glm::dvec3 altitude = glm::normalize(edgeTriangle.GetAltitude(3)); 
    const glm::dvec3 edgeMidpoint = mesh.calc_edge_midpoint(heh);
    const glm::dvec3 edgeVector = mesh.calc_edge_vector(heh);
    const glm::dvec3 altitude = glm::normalize(glm::cross(edgeVector, object.Normal(edgeMidpoint)));
    const glm::dvec3 equilateralPoint = getEtp(P0, P1, altitude);
    assert(std::abs(glm::distance(equilateralPoint, P0) - glm::distance(P0, P1)) <= 0.001f);
    assert(std::abs(glm::distance(equilateralPoint, P1) - glm::distance(P0, P1)) <= 0.001f);
    
    glm::dvec3 newPoint = object.Project(equilateralPoint);
    assert(std::abs(object.Evaluate(newPoint)) <= 0.0001f);
    
    const float sideLength = getRoc(newPoint) * rho;
    const auto interpolatedNormal = (object.Normal(edgeMidpoint) + object.Normal(equilateralPoint)) / 2.;
    const glm::dvec3 isoScelesPoint = getItp(P0, P1, glm::normalize(glm::cross(edgeVector, interpolatedNormal)), sideLength);
    assert(std::abs(glm::distance(isoScelesPoint, P0) - sideLength) <= 0.001f);
    assert(std::abs(glm::distance(isoScelesPoint, P1) - sideLength) <= 0.001f);
    
    newPoint = object.Project(isoScelesPoint);
    assert(std::abs(object.Evaluate(newPoint)) <= 0.0001f);
    //glm::dvec3 newPoint = equilateralPoint;

    {
        // Check 1: new edges and the expanded edge must have an angle of at least 45 degrees
        constexpr double requiredAngle = glm::radians(45.0);
        if (glm::angle(glm::normalize(newPoint - P0), glm::normalize(P1 - P0)) <= requiredAngle ||
            glm::angle(glm::normalize(newPoint - P1), glm::normalize(P0 - P1)) <= requiredAngle)
            return false;
    }

    {
        // Check 2: no existing triangles within r range
        if (kdTreeIsDirty)
            kdTree.buildIndex();

        const Triangle longestSidedTriangle(OpenMesh::SmartFaceHandle(longestSidedFace.idx(), &mesh), mesh);
        const double q = longestSidedTriangle.GetDistanceFrom(newPoint);

        const Triangle newTriangle(P0, P1, newPoint);
        const double newTriangleLongestSide = newTriangle.GetLongestSide();
        double longestSideNew = std::max(longestSide, newTriangleLongestSide);

        const double t = 2.0 * longestSideNew / 3.0 + q;
        const double v = longestSide / 2.0;
        const double r = sqrt(t * t + v * v);

        std::vector<std::pair<unsigned int, double>> matches;
        kdTree.radiusSearch(glm::value_ptr(newTriangle.GetCentroid()), r, matches, nanoflann::SearchParams());

        for (auto &match : matches)
        {
            for (const auto& closeFace :
                mesh.vf_range(OpenMesh::VertexHandle(match.first)))
            {
                auto closeTriangle = Triangle(closeFace, mesh);
                longestSideNew = std::max(closeTriangle.GetLongestSide(), newTriangleLongestSide);
                if (longestSideNew / 2.0 > closeTriangle.GetDistanceFrom(newPoint))
                {
                    return false;
                }
            }
        }
    }

    const auto pointA = mesh.to_vertex_handle(heh);
    const auto pointB = mesh.from_vertex_handle(heh);
    const double longestSideHere = Triangle{ mesh.point(pointA), mesh.point(pointB), newPoint }.GetLongestSide();

    newFace = addNewFace(pointA, pointB, newPoint, longestSideHere);
    mesh.data(newFace).faceCreationMethod = FaceCreationMethod::IsoscelesGrowing;

    return true;
}

bool Implicit::CurvatureTessellator::applyEarCutting(const OpenMesh::SmartFaceHandle& newFace)
{
    bool earCut = false;

    auto heh = newFace.halfedge();

    while (!heh.edge().is_boundary())
        heh = heh.next();
    
    if (!heh.is_boundary())
        heh = heh.opp();

    const float cutAngle = 70.0f;
    const auto heh_next = heh.next();
    const auto heh_prev = heh.prev();

    if (std::abs(glm::degrees(mesh.calc_sector_angle(heh_next))) <= cutAngle)
    {
        const auto heh_next_next = heh_next.next();
        mesh.data(heh_next.edge()).grownAlready = true;
        mesh.data(heh_next_next.edge()).grownAlready = true;
        auto newFace = mesh.add_face(heh.to(), heh_next.to(), heh_next_next.to());
        if (newFace == GlmTriMesh::FaceHandle()) {
            throw std::runtime_error("Failed to create new face");
        }
        mesh.data(newFace).faceCreationMethod = FaceCreationMethod::EarCutting;
        earCut = true;
    }

    if (glm::degrees(std::abs(mesh.calc_sector_angle(heh_prev))) <= cutAngle)
    {
        mesh.data(heh_prev.edge()).grownAlready = true;
        mesh.data(heh.edge()).grownAlready = true;
        auto newFace = mesh.add_face(heh_prev.from(), heh.from(), heh.to());
        if (newFace == GlmTriMesh::FaceHandle()) {
            throw std::runtime_error("Failed to create new face");
        }
        mesh.data(newFace).faceCreationMethod = FaceCreationMethod::EarCutting;
        earCut = true;
    }
    
    return earCut;
}

OpenMesh::VertexHandle Implicit::CurvatureTessellator::addGapVertex(OpenMesh::VertexHandle vertex, bool rebuildKdTree)
{
    auto gapVertex = gap.add_vertex(mesh.point(vertex));
    gap.data(gapVertex).connectedVertex = vertex;
    mesh.data(vertex).connectedVertex = gapVertex;
    if (rebuildKdTree)
        gapKdTree.buildIndex();

    return gapVertex;
}

void Implicit::CurvatureTessellator::removeGapVertex(OpenMesh::VertexHandle vertex)
{
    auto gapVertex = mesh.data(vertex).connectedVertex;
    gap.delete_vertex(gapVertex);
}

OpenMesh::VertexHandle Implicit::CurvatureTessellator::computeClosestNeighbour(OpenMesh::SmartHalfedgeHandle heh)
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
            if (!gap.status(gapVertex).deleted())
            {
                auto meshVertex = gap.data(gapVertex).connectedVertex;
                if (isNeighbour(heh, meshVertex))
                {
                    closestNeighbour = meshVertex;
                    break;
                }
            }
        }

        if (matches.size() == gap.n_vertices())
        {
            break;
        }
    } while (!closestNeighbour.is_valid());
    return closestNeighbour;
}

bool Implicit::CurvatureTessellator::isConvex(OpenMesh::SmartHalfedgeHandle toHalfedge)
{
    const double sectorAngle = mesh.calc_sector_angle(toHalfedge);
    const bool isConvex = 0.0 <= sectorAngle && sectorAngle <= glm::radians(180.0);
    return isConvex;
}

bool Implicit::CurvatureTessellator::isNeighbour(OpenMesh::SmartHalfedgeHandle toHalfedge, OpenMesh::VertexHandle vertex)
{
    const bool isConvex = this->isConvex(toHalfedge);
    const auto planePoint = mesh.point(toHalfedge.to());
    const auto pointNormal = object.Normal(planePoint);
    const Plane pv1{ planePoint, pointNormal, mesh.calc_edge_vector(toHalfedge) };
    const Plane pv2{ planePoint, pointNormal, mesh.calc_edge_vector(toHalfedge.next())  };
    const bool isAbovePv1 = pv1.IsAbove( mesh.point(vertex),  mesh.calc_edge_length(toHalfedge) / 10.0 );
    const bool isAbovePv2 = pv2.IsAbove( mesh.point(vertex),  mesh.calc_edge_length(toHalfedge.next()) / 10.0 );
    if (isConvex) {
        return isAbovePv1 && isAbovePv2;
    }
    else {
        return isAbovePv1 || isAbovePv2;
    }

}

bool Implicit::CurvatureTessellator::isBridge(OpenMesh::VertexHandle vertex)
{
    auto closestNeigbour = mesh.data(vertex).closestNeighbour;
    return mesh.data(closestNeigbour).closestNeighbour == vertex;
}

bool Implicit::CurvatureTessellator::SmallPolygonFilling(OpenMesh::FaceHandle gap)
{
    if(mesh.valence(gap) != 4)
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

bool Implicit::CurvatureTessellator::SubdivisionOnBridges(OpenMesh::FaceHandle gap)
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

bool Implicit::CurvatureTessellator::XFilling(OpenMesh::FaceHandle gap)
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
                        mesh.data(v1).closestNeighbour = computeClosestNeighbour(heh.prev());
                    if(mesh.data(v4).closestNeighbour == v1)
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

bool Implicit::CurvatureTessellator::EarFilling(OpenMesh::FaceHandle gap)
{
    return false;
}

bool Implicit::CurvatureTessellator::ConvexPolygonFilling(OpenMesh::FaceHandle gap)
{
    return false;
}

bool Implicit::CurvatureTessellator::RelaxedEarFilling(OpenMesh::FaceHandle gap)
{
    return false;
}

bool Implicit::CurvatureTessellator::ConcaveVertexBisection(OpenMesh::FaceHandle gap)
{
    return false;
}

void Implicit::CurvatureTessellator::computeClosestNeighbours()
{
    OpenMesh::SmartHalfedgeHandle heh_start = FindBoundaryHalfEdge( mesh );

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

    closestNeighboursComputed = true;

    auto initial_gap = mesh.add_face(gapVertices);
    if (!initial_gap.is_valid())
        throw std::runtime_error("Can't create initial gap");

    gaps.push_back(initial_gap);
}

bool Implicit::CurvatureTessellator::RunGrowingIterations(int iterations)
{
    bool meshChanged = false;

    if (mesh.n_faces() == 0) {
        GenerateSeedTriangle();
        meshChanged = true;
    }

    int iteration = 0;
    for (auto edge = mesh.edges_sbegin();
        edge != mesh.edges_end() && iteration < iterations;
        ++edge)
    {
        OpenMesh::SmartFaceHandle newFace;
        if (expandEdge(*edge, newFace))
        {
            applyEarCutting(newFace);
            meshChanged = true;
            ++iteration;
        }
    }

    if (!meshChanged && !closestNeighboursComputed)
        computeClosestNeighbours();

    if (visitor)
        visitor->get().IterationEnded(meshChanged);

    return !meshChanged; 
}

bool Implicit::CurvatureTessellator::RunFillingIterations(int iterations)
{
    if (gaps.empty())
        return false;

    auto gap = gaps.front();
    gaps.pop_front();

    const bool meshChanged =
        SmallPolygonFilling(gap) ||
        SubdivisionOnBridges(gap) ||
        XFilling(gap) ||
        EarFilling(gap) ||
        RelaxedEarFilling(gap) ||
        ConcaveVertexBisection(gap);

    return meshChanged;
}
