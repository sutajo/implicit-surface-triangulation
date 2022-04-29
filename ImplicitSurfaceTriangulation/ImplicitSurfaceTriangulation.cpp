#include "ImplicitSurfaceTriangulation.hpp"
#include <glm/gtx/vector_angle.hpp>

Implicit::CurvatureTessellator::CurvatureTessellator(Object& object) 
    : object(object), visitor()
{
    mesh.request_face_normals();
    mesh.request_vertex_normals();
}

Implicit::CurvatureTessellator::CurvatureTessellator(Object& object, Visitor& visitor) 
    : object(object), visitor(visitor)
{
    mesh.request_face_normals();
    mesh.request_vertex_normals();
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

    addNewFace(seedPoint, nextPoint, finalPoint);

    if (visitor)
        visitor->get().SeedTriangleGenerated(Triangle{seedPoint, nextPoint, finalPoint});
}

void Implicit::CurvatureTessellator::ComputeMesh()
{
    
}

const GlmMesh& Implicit::CurvatureTessellator::GetMesh()
{
    return mesh;
}

const Aabb Implicit::CurvatureTessellator::ComputeBoundingBox()
{
    GlmMeshKdTree::BoundingBox bb;
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

    const double height = sqrt(equalSideLength*equalSideLength - pow(midDistance, 2.0));
    return midPoint + height * normalizedTangent;
}

void Implicit::CurvatureTessellator::addNewFace(const glm::dvec3& pointA, const glm::dvec3& pointB, const glm::dvec3& pointC)
{
    const double longestSideHere = Triangle{ pointA, pointB, pointC }.GetLongestSide();
    auto faceHandle = mesh.add_face(mesh.add_vertex(pointA), mesh.add_vertex(pointB), mesh.add_vertex(pointC));
    if (faceHandle == GlmMesh::FaceHandle()) {
        mesh.clean();
        throw std::runtime_error("Failed to create new face");
    }

    kdTreeIsDirty = true;

    if (longestSideHere > longestSide)
    {
        longestSide = longestSideHere;
        longestSidedFace = faceHandle;
    }
}

OpenMesh::SmartFaceHandle Implicit::CurvatureTessellator::addNewFace(const OpenMesh::VertexHandle& pointA, const OpenMesh::VertexHandle& pointB, const glm::dvec3& pointC)
{
    const double longestSideHere = Triangle{ mesh.point(pointA), mesh.point(pointB), pointC }.GetLongestSide();
    
    auto newVertex = mesh.add_vertex(pointC);
    if (newVertex == GlmMesh::VertexHandle())
        throw std::runtime_error("Failed to create new vertex");

    auto faceHandle = mesh.add_face(pointA, pointB, newVertex);
    if (faceHandle == GlmMesh::FaceHandle()) {
        mesh.delete_vertex(newVertex);
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

bool Implicit::CurvatureTessellator::expandEdge(OpenMesh::SmartEdgeHandle edge, OpenMesh::SmartFaceHandle& newFace)
{
    const bool grownAlready = std::exchange(mesh.data(edge).grownAlready, true);
    if (grownAlready)
        return false; // The edge was grown already

    if (!mesh.is_boundary(edge))
        throw std::runtime_error("Only boundary edges can be expanded");

    Triangle edgeTriangle;
    
    // Get the face corresponding to the boundary edge
    OpenMesh::SmartFaceHandle face;
    auto heh = mesh.halfedge_handle(edge, 0);
    if(mesh.is_boundary(heh))
        heh = mesh.halfedge_handle(edge, 1);

    face = mesh.face_handle(heh);
#ifdef DEBUG
    int idx = face.idx();
#endif // DEBUG
    edgeTriangle.a = mesh.point(edge.v0());
    edgeTriangle.b = mesh.point(edge.v1());
    edgeTriangle.c = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh)));

    const glm::dvec3 altitude = glm::normalize(edgeTriangle.GetAltitude(3)); 
    const glm::dvec3 equilateralPoint = getEtp(edgeTriangle.a, edgeTriangle.b, altitude);
    assert(std::abs(glm::distance(equilateralPoint, edgeTriangle.a) - glm::distance(edgeTriangle.a, edgeTriangle.b)) <= 0.001f);
    assert(std::abs(glm::distance(equilateralPoint, edgeTriangle.b) - glm::distance(edgeTriangle.a, edgeTriangle.b)) <= 0.001f);
    
    glm::dvec3 newPoint = object.Project(equilateralPoint);
    assert(std::abs(object.Evaluate(newPoint)) <= 0.0001f);
    
    const float sideLength = getRoc(newPoint) * rho;
    const glm::dvec3 isoScelesPoint = getItp(edgeTriangle.a, edgeTriangle.b, altitude, sideLength);
    assert(std::abs(glm::distance(isoScelesPoint, edgeTriangle.a) - sideLength) <= 0.001f);
    assert(std::abs(glm::distance(isoScelesPoint, edgeTriangle.b) - sideLength) <= 0.001f);
    
    newPoint = object.Project(isoScelesPoint);
    assert(std::abs(object.Evaluate(newPoint)) <= 0.0001f);

    {
        // Check 1: new edges and the expanded edge must have an angle of at least 45 degrees
        constexpr double requiredAngle = glm::radians(45.0);
        if (glm::angle(glm::normalize(newPoint - edgeTriangle.a), glm::normalize(edgeTriangle.b - edgeTriangle.a)) <= requiredAngle ||
            glm::angle(glm::normalize(newPoint - edgeTriangle.b), glm::normalize(edgeTriangle.a - edgeTriangle.b)) <= requiredAngle)
            return false;
    }

    {
        // Check 2: no existing triangles within r range
        if (kdTreeIsDirty)
            kdTree.buildIndex();

        const Triangle longestSidedTriangle(OpenMesh::SmartFaceHandle(longestSidedFace.idx(), &mesh), mesh);
        const double q = longestSidedTriangle.GetDistanceFrom(newPoint);

        const Triangle newTriangle(edgeTriangle.a, edgeTriangle.b, newPoint);
        const double newTriangleLongestSide = newTriangle.GetLongestSide();
        double longestSideNew = std::max(longestSide, newTriangleLongestSide);

        const double t = 2.0 * longestSideNew / 3.0 + q;
        const double v = longestSide / 2.0;
        const double r = sqrt(t * t + v * v);

        std::vector<std::pair<unsigned int, double>> matches;
        kdTree.radiusSearch(&newTriangle.GetCentroid()[0], r, matches, nanoflann::SearchParams());

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

    newFace = addNewFace(mesh.to_vertex_handle(heh), mesh.from_vertex_handle(heh), newPoint);
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
        if (newFace == GlmMesh::FaceHandle()) {
            throw std::runtime_error("Failed to create new face");
        }
        mesh.data(newFace).createdByEarCutting = true;
        earCut = true;
    }

    if (glm::degrees(std::abs(mesh.calc_sector_angle(heh.prev()))) <= cutAngle)
    {
        mesh.data(heh_prev.edge()).grownAlready = true;
        mesh.data(heh.edge()).grownAlready = true;
        auto newFace = mesh.add_face(heh_prev.from(), heh.from(), heh.to());
        if (newFace == GlmMesh::FaceHandle()) {
            throw std::runtime_error("Failed to create new face");
        }
        mesh.data(newFace).createdByEarCutting = true;
        earCut = true;
    }
    
    return earCut;
}

bool Implicit::CurvatureTessellator::RunIterations(int iterations)
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

    if (visitor)
        visitor->get().IterationEnded(meshChanged);

    return !meshChanged; 
}