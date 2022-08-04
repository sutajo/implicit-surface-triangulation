#include "GrowingPhase.hpp"
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace Implicit::Tessellation;
using namespace OpenMesh;
using namespace glm;
using namespace std;
using namespace nanoflann;

GrowingPhase::GrowingPhase(GlmPolyMesh& mesh, Object& object) : 
    Phase(mesh, mesh), 
    mesh(mesh), 
    object(object), 
    kdTreeAdaptor(mesh), 
    kdTree{ 3, kdTreeAdaptor }
{
    mesh.request_face_normals();
    mesh.request_vertex_normals();
}

void GrowingPhase::generateSeedTriangle()
{
    const dvec3 seedPoint = object.GetStartVertex();
    const dvec3 normal = object.Normal(seedPoint);

    dvec3 t, b;
    object.getTangentSpace(normal, t, b);

    const double roc = getRoc(seedPoint);
    const double sideLength = roc * rho;

    const dvec3 nextPoint = object.Project(seedPoint + t * sideLength);
    const dvec3 finalPoint = Triangle::getEtp(seedPoint, nextPoint, b); // object.Project();

    if (!object.GetBoundingBox().contains(nextPoint) ||
        !object.GetBoundingBox().contains(finalPoint))
        throw runtime_error("Failed to generate seed triangle: vertices fall out of the bounding box");

    auto newFace = addNewFace(seedPoint, nextPoint, finalPoint);
    mesh.data(newFace).faceCreationMethod = FaceCreationMethod::Seed;
}

double GrowingPhase::getRoc(const dvec3& point) const
{
	double k1, k2;
	object.Curvature(point, k1, k2);
	return 1.0f / std::max(abs(k1), abs(k2));
}

SmartFaceHandle GrowingPhase::addNewFace(const dvec3& pointA, const dvec3& pointB, const dvec3& pointC)
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

SmartFaceHandle GrowingPhase::addNewFace(const VertexHandle& pointA, const VertexHandle& pointB, const dvec3& pointC, double newTriangleLongestSide)
{
    auto newVertex = mesh.add_vertex(pointC);
    if (newVertex == GlmTriMesh::VertexHandle())
        throw runtime_error("Failed to create new vertex");

    auto faceHandle = mesh.add_face(pointA, pointB, newVertex);
    if (faceHandle == GlmTriMesh::FaceHandle()) {
        mesh.delete_vertex(newVertex);
        throw runtime_error("Failed to create new face");
    }

    kdTreeIsDirty = true;

    if (newTriangleLongestSide > longestSide)
    {
        longestSide = newTriangleLongestSide;
        longestSidedFace = faceHandle;
    }

    return faceHandle;
}

bool GrowingPhase::expandEdge(OpenMesh::SmartEdgeHandle edge, OpenMesh::SmartFaceHandle& newFace)
{
    const bool grownAlready = exchange(mesh.data(edge).grownAlready, true);
    if (grownAlready)
        return false; // The edge was grown already

    if (!mesh.is_boundary(edge))
        throw runtime_error("Only boundary edges can be expanded");

    // Get the face corresponding to the boundary edge
    auto heh = mesh.halfedge_handle(edge, 0);
    if (mesh.is_boundary(heh))
        heh = mesh.halfedge_handle(edge, 1);

    const dvec3 P0 = mesh.point(edge.v0());
    const dvec3 P1 = mesh.point(edge.v1());

    // const glm::dvec3 altitude = glm::normalize(edgeTriangle.GetAltitude(3)); 
    const dvec3 edgeMidpoint = mesh.calc_edge_midpoint(heh);
    const dvec3 edgeVector = mesh.calc_edge_vector(heh);
    const dvec3 altitude = glm::normalize(glm::cross(edgeVector, object.Normal(edgeMidpoint)));
    const dvec3 equilateralPoint = Triangle::getEtp(P0, P1, altitude);
    assert(abs(distance(equilateralPoint, P0) - distance(P0, P1)) <= 0.001f);
    assert(abs(distance(equilateralPoint, P1) - distance(P0, P1)) <= 0.001f);

    dvec3 newPoint = object.Project(equilateralPoint);
    assert(abs(object.Evaluate(newPoint)) <= 0.0001f);

    const float sideLength = getRoc(newPoint) * rho;
    const auto interpolatedNormal = (object.Normal(edgeMidpoint) + object.Normal(equilateralPoint)) / 2.;
    const dvec3 isoScelesPoint = Triangle::getItp(P0, P1, normalize(cross(edgeVector, interpolatedNormal)), sideLength);
    //assert(std::abs(glm::distance(isoScelesPoint, P0) - sideLength) <= 0.001f);
    //assert(std::abs(glm::distance(isoScelesPoint, P1) - sideLength) <= 0.001f);

    newPoint = object.Project(isoScelesPoint);
    assert(abs(object.Evaluate(newPoint)) <= 0.0001f);
    //glm::dvec3 newPoint = equilateralPoint;

    {
        // Check 1: new edges and the expanded edge must have an angle of at least 45 degrees
        constexpr double requiredAngle = radians(45.0);
        if (angle(normalize(newPoint - P0), normalize(P1 - P0)) <= requiredAngle ||
            angle(glm::normalize(newPoint - P1), normalize(P0 - P1)) <= requiredAngle)
            return false;
    }

    {
        // Check 2: no existing triangles within r range
        if (kdTreeIsDirty)
            kdTree.buildIndex();

        const Triangle longestSidedTriangle(SmartFaceHandle(longestSidedFace.idx(), &mesh), mesh);
        const double q = longestSidedTriangle.GetDistanceFrom(newPoint);

        const Triangle newTriangle(P0, P1, newPoint);
        const double newTriangleLongestSide = newTriangle.GetLongestSide();
        double longestSideNew = std::max(longestSide, newTriangleLongestSide);

        const double t = 2.0 * longestSideNew / 3.0 + q;
        const double v = longestSide / 2.0;
        const double r = sqrt(t * t + v * v);

        vector<pair<unsigned int, double>> matches;
        kdTree.radiusSearch(glm::value_ptr(newTriangle.GetCentroid()), r, matches, SearchParams());

        for (auto& match : matches)
        {
            for (const auto& closeFace :
                mesh.vf_range(VertexHandle(match.first)))
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

bool GrowingPhase::applyEarCutting(const SmartFaceHandle& newFace)
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

    if (abs(degrees(mesh.calc_sector_angle(heh_next))) <= cutAngle)
    {
        const auto heh_next_next = heh_next.next();
        mesh.data(heh_next.edge()).grownAlready = true;
        mesh.data(heh_next_next.edge()).grownAlready = true;
        auto newFace = mesh.add_face(heh.to(), heh_next.to(), heh_next_next.to());
        if (newFace == GlmTriMesh::FaceHandle()) {
            throw runtime_error("Failed to create new face");
        }
        mesh.data(newFace).faceCreationMethod = FaceCreationMethod::EarCutting;
        earCut = true;
    }

    if (degrees(abs(mesh.calc_sector_angle(heh_prev))) <= cutAngle)
    {
        mesh.data(heh_prev.edge()).grownAlready = true;
        mesh.data(heh.edge()).grownAlready = true;
        auto newFace = mesh.add_face(heh_prev.from(), heh.from(), heh.to());
        if (newFace == GlmTriMesh::FaceHandle()) {
            throw runtime_error("Failed to create new face");
        }
        mesh.data(newFace).faceCreationMethod = FaceCreationMethod::EarCutting;
        earCut = true;
    }

    return earCut;
}

void GrowingPhase::Start()
{
    Phase::Start();
    generateSeedTriangle();
}

void GrowingPhase::Run()
{
    for (auto edge = mesh.edges_sbegin();
        edge != mesh.edges_end();
        ++edge)
    {
        SmartFaceHandle newFace;
        if (expandEdge(*edge, newFace))
        {
            applyEarCutting(newFace);
        }
    }

    meshChanged = false; // That's a lie
}

void GrowingPhase::RunIterations(int iterations)
{
    meshChanged = false;

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
}

bool GrowingPhase::Completed() const
{
	return !meshChanged;
}

void GrowingPhase::SetRho(double rho)
{
    this->rho = rho;
}

double GrowingPhase::GetLongestTriangleSide() const
{
    return longestSide;
}