#include <algorithm>
#include <numeric>
#include <set>
#include <optional>
#include <variant>
#include <spdlog/spdlog.h>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "cdt.hh"
#include "util.hh"
#include "types.hh"
#include "constants.hh"

using namespace std;
using namespace Geometry;
using namespace autodiff;
using namespace nanoflann;
using namespace spdlog;

static TriMesh
GrowingMeshToTrimesh(const GrowingMesh &Mesh)
{
    TriMesh OutputMesh;
    Geometry::PointVector Points;
    Points.reserve(Mesh.n_vertices());
    for (const auto &point : Mesh.vertices())
    {
        const auto p = Mesh.point(point);
        Points.emplace_back(p[0], p[1], p[2]);
    }
    OutputMesh.setPoints(Points);
    for (const auto &face : Mesh.faces())
    {
        const auto vertices = face.vertices().to_array<3>();
        OutputMesh.addTriangle(vertices[0].idx(), vertices[1].idx(), vertices[2].idx());
    }
    return OutputMesh;
}

// Él kiterjesztés randomizálás
// Speciális esetnél háromszög oldalainak vizsgálata, többi háromszöghöz való távolság vizsgálata

static const double l = sqrt(3.0) / 2.0;

struct PointCompare
{
    bool operator()(const Point3D &lhs, const Point3D &rhs) const
    {
        return std::tuple<double, double, double>(lhs[0], lhs[1], lhs[2]) < std::tuple<double, double, double>(rhs[0], rhs[1], rhs[2]);
    }
};

#define assert_near(X, Y, eps) assert(std::abs((X) - (Y)) < (eps))

// Nem lehet létrehozni az új háromszöget, mert túl közel esne egy létező háromszöghöz.
struct TrianglesTooNear
{
};

// Az előző egy speciális esete, amikor az új csúcs majdnem ugyanoda esik mint egy létező, ilyenkor.
// mégis létre lehet hozni a háromszöget
struct UseExistingVertices
{
    OpenMesh::VertexHandle vertices[3];
    double LongestSide;
    OpenMesh::SmartEdgeHandle oppositeEdge;
};

// Az új háromszög eléggé távol van, felvehető az új csúcs. Az KD-fát újra kell építeni.
struct AddNewVertex
{
    double LongestSide;
};

std::vector<GrowingMesh::VertexHandle> find_common_neighbors(GrowingMesh &mesh, GrowingMesh::VertexHandle &v1, GrowingMesh::VertexHandle &v2)
{
    std::vector<GrowingMesh::VertexHandle> common_neighbors;
    std::unordered_map<GrowingMesh::VertexHandle, bool> candidates;

    for (GrowingMesh::VertexVertexIter it = mesh.vv_iter(v1); it.is_valid(); ++it)
    {
        candidates[*it] = true;
    }

    for (GrowingMesh::VertexVertexIter it = mesh.vv_iter(v2); it.is_valid(); ++it)
    {
        if (candidates.find(*it) != candidates.end())
            common_neighbors.push_back(*it);
    }

    return common_neighbors;
}

using CheckResult = variant<TrianglesTooNear, UseExistingVertices, AddNewVertex>;

static CheckResult
CheckNeighbourTriangles(
    const GrowingState &state,
    const array<ImplicitTriangulation::Vector3D, 3> &newTriangle,
    const OpenMesh::SmartEdgeHandle grownEdge)
{
    const auto c = ProjectToTriangle(newTriangle.back(), state.longestSidedTriangle.FaceHandle, state.mesh);
    const auto q = (newTriangle.back() - c).norm();

    const auto newTriangleLongestSide = LongestSide(newTriangle[0], newTriangle[1], newTriangle[2]);
    const auto longestSide = max(newTriangleLongestSide, state.longestSidedTriangle.longestSide);
    const auto t = 2.0 * longestSide / 3.0 + q;
    const auto v = state.longestSidedTriangle.longestSide / 2.0;
    const auto r = sqrt(t * t + v * v);

    vector<pair<size_t, double>> ret_matches;
    SearchParams params;
    params.sorted = true;

    // Az új háromszög súlypontot
    const auto Centroid = accumulate(newTriangle.begin(), newTriangle.end(), ImplicitTriangulation::Vector3D(0, 0, 0)) / 3.0;
    // Sugáron belül eső csúcsok kiszűrése
    const auto matchCount = state.kdtree.radiusSearch(Centroid.data(), r, ret_matches, params);

    // Végigmegyünk a sugáron belül lévő pontokon
    for (const auto &match : ret_matches)
    {
        // Végigmegyünk a ponthoz tartozó háromszögeken
        for (const auto &face :
             state.mesh.vf_range(OpenMesh::VertexHandle(match.first)))
        {
            const auto from_vertex = state.mesh.from_vertex_handle(state.mesh.halfedge_handle(grownEdge, 0));
            const auto to_vertex = state.mesh.to_vertex_handle(state.mesh.halfedge_handle(grownEdge, 0));

            const auto face_contains = [&](const OpenMesh::VertexHandle &vertex, const OpenMesh::SmartFaceHandle &face) -> bool
            {
                return face.vertices().any_of([&](const OpenMesh::VertexHandle &v)
                                              { return v == vertex; });
            };

            const auto edge_contains = [&](const OpenMesh::VertexHandle &vertex, const OpenMesh::SmartEdgeHandle &edge) -> bool
            {
                return face.edges().any_of([&](const OpenMesh::SmartEdgeHandle &e)
                                           { return e.v0() == vertex || e.v1() == vertex; });
            };

            // A háromszög tartalmazza a növelési él egyik csúcsát
            const auto contains_to = face_contains(to_vertex, face);
            // A háromszög tartalmazza a növelési él másik csúcsát
            const auto contains_from = face_contains(from_vertex, face);

            // Ez azt jelenti, hogy a növeléshez használt élt tartalmazza a háromszög, átugorhatjuk,
            // mert ha ilyen esetekben is vizsgálnánk a közelséget, akkor ilyen háló nem alakulhatnak ki:
            //                ______
            //              /\     /
            //             /  \   /
            //            /____\ /
            //
            if (contains_to && contains_from)
                continue;

            // Speciális eset:
            //   _______C
            //   \     / \
            //    \   /   \ Kiterjesztett él
            //     \ /     \
            //      P
            // Ha:
            //  - a háromszög egyik csúcsa a kiterjesztett él egyik csúcsa és
            //  - az új csúcs közel van a háromszög egyik csúcsához

            if (contains_to || contains_from)
            {
                auto vertexIter = face.vertices().begin();
                for (; vertexIter != face.vertices().end(); ++vertexIter)
                {
                    // Meglévő háromszög pont és az új pont távolsága
                    const auto d = (state.mesh.point(*vertexIter) - newTriangle.back()).norm();
                    if (d < 0.65e-2)
                        break;
                }
                const auto CanUseExistingVertices = vertexIter != face.vertices().end();
                if (CanUseExistingVertices)
                {
                    const auto commonVertex = contains_to ? to_vertex : from_vertex;
                    const auto heh = state.mesh.find_halfedge(commonVertex, *vertexIter);
                    assert(heh.is_valid());
                    OpenMesh::SmartEdgeHandle oppositeEdge = state.mesh.edge_handle(heh);
                    assert(oppositeEdge.is_valid());
                    return UseExistingVertices{{to_vertex, from_vertex, *vertexIter}, longestSide, oppositeEdge};
                }
            }

            const auto FaceVertices = face.vertices().to_array<3>();

            const auto A = state.mesh.point(FaceVertices[0]);
            const auto B = state.mesh.point(FaceVertices[1]);
            const auto C = state.mesh.point(FaceVertices[2]);

            const double longestSide =
                max(newTriangleLongestSide, LongestSide(A, B, C));

            const auto c = ProjectToTriangle(newTriangle.back(), face, state.mesh);
            const auto q = (newTriangle.back() - c).norm();

            if (q < longestSide / 2.0)
                return TrianglesTooNear();
        }
    }

    return AddNewVertex{longestSide};
}

//
// - Fül levágás https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
// Vége, ha nem végezhető több művelet egyik listában lévő élen se
// Reprezentáció: csúcsokat tartalmazó vektor, él: indexpár
// Akkor alkalmazandó, amikor két szomszédos él külső szöge kisebb mint 70 fok
// A két él csúcsai egy új háromszöget definiálnak. A 70 fok nem tudományos úton jött ki, egyszerű heurisztika,
// hogy egyenlő oldalú háromszögeken kívül más háromszögek is létrejöhessenek.
//
static bool
EarCutting(const GrowingState &state)
{
    bool OperationTookPlace = false;

    auto initHalfEdge = state.mesh.halfedge_handle(0);
    if (!state.mesh.is_boundary(initHalfEdge))
    {
        initHalfEdge = state.mesh.opposite_halfedge_handle(initHalfEdge);
    }

    auto halfedge = initHalfEdge;

    std::set<OpenMesh::VertexHandle> verticesForDeletion;

    do
    {
        const auto angle = state.mesh.calc_sector_angle(halfedge) * 180.0 / M_PI;
        spdlog::info("Sector angle: {} degrees", angle);

        if (angle > 110.0) // A külső szög kisebb mint 70 fok
        {
            const auto vertexToDelete = state.mesh.to_vertex_handle(halfedge);
            spdlog::info("Ear cutting: Deleting vertex {}", vertexToDelete.idx());
            state.mesh.delete_vertex(vertexToDelete);
        }

        halfedge = state.mesh.next_halfedge_handle(halfedge);
    } while (initHalfEdge != halfedge);

    return OperationTookPlace;
}

static bool
IsoscelesTriangleGrowing(
    function<dual(
        const dual &x,
        const dual &y,
        const dual &z)>
        ScalarFunction,
    const double isoLevel,
    const double rho,
    GrowingState &state,
    const array<ImplicitTriangulation::Vector3D, 2> &BoundingBox)
{
    // Egyenlő szárú háromszög növelés:
    // Az élre merőleges, a meglévő felosztáson kívül található félsíkban választunk egy p pontot úgy,
    // hogy az él két végpontja (u,v) és p egy egyenlő oldalú háromszöget alkosson. p-t levetítjük a felületre
    // az u,v,p pontokkal megbecsüljük R-t, a lokális görbületi sugarat.
    // A p pontot ismét betesszük az (u,v)-ra merőleges síkba, de ezúttal úgy, hogy d(u,p) = d(p,v) = R * Rho
    // Ezután p-t újra levetítjük a felületre
    // 2 ellenőrző teszt:
    // - mindkét új élnek legalább 45 fokos szöget kell bezárnia az (u,v) éllel
    // - az új háromszögnek nem szabad túlságosan megközelítenie a meglévő háromszögeket. Ehhez az ellenőrzéshez végig kell
    // iterálni az új háromszög súlypontjától r távolságra eső háromszögeken (r kiszámítását lásd később). Ehhez egy KD-fát kell majd
    // építeni, hogy gyorsan lekérdezhetők legyenek a szomszédok.
    // Ha van köztük olyan T' háromszög, ami közelebb van (bármely pontja közelebb van) az új T háromszöghöz, mint T és T' összesített éleiből a leghosszabbnak a fele,
    // akkor az új háromszög nem elfogadható
    // Az egyenlő háromszög növelés műveletet minden aktív élen legfeljebb egyszer van értelme megpróbálni (bool flag jelezze, hogy volt-e már próbálva)

    bool success = false;

    for (auto edge = state.mesh.edges_begin(); edge != state.mesh.edges_end(); ++edge)
    {
        // GrowingMeshToTrimesh(state.mesh).writeOBJ("testmesh.obj");
        for (auto &edge : state.mesh.edges())
            spdlog::debug("Edge({}): {}<->{}", edge.idx(), edge.v0().idx(), edge.v1().idx());
        spdlog::debug("---------");

        spdlog::info("Growing edge {}", edge->idx());

        auto &edgeData = state.mesh.data(*edge);
        if (edgeData.GrownAlready)
        {
            spdlog::info("Edge {} grown already", edge->idx());
            continue;
        }

        edgeData.GrownAlready = true;

        // Csak a mesh szélén lévő éleket van értelme kiterjeszteni
        assert(state.mesh.is_boundary(*edge));

        // Az élhez tartozó háromszög megkeresése, ami a mesh része
        const auto innerFaceHalfEdge = state.mesh.is_boundary(state.mesh.halfedge_handle(*edge, 0))
                                           ? state.mesh.halfedge_handle(*edge, 1)
                                           : state.mesh.halfedge_handle(*edge, 0);

        const auto innerFace = state.mesh.face_handle(innerFaceHalfEdge);

        auto &innerFaceData = state.mesh.data(innerFace);
        const auto vertices = innerFace.vertices().to_array<3>();

        const auto handle_u = edge->v0();
        const auto &u = state.mesh.point(handle_u);
        const auto handle_v = edge->v1();
        const auto &v = state.mesh.point(handle_v);
        const auto handle_q = state.mesh.to_vertex_handle(state.mesh.next_halfedge_handle(innerFaceHalfEdge));
        assert(handle_u != handle_q && handle_v != handle_q);
        const auto &q = state.mesh.point(handle_q);

        const auto uv = v - u;
        const auto uvd = uv.norm();
        const auto uvn = uv / uvd;
        const auto height = l * uvd;

        const auto m = (u + v) / 2.0;

        const auto qu = u - q;
        const auto qv = v - q;
        const auto normal = qv.cross(qu).normalized();
        const auto perpv = normal.cross(uv).normalized() * height;

        auto p = ProjectPointToSurface(ScalarFunction, isoLevel, m + perpv);

        const auto normal_at_u = GradientAt(ScalarFunction, u);
        const auto normal_at_v = GradientAt(ScalarFunction, v);
        const auto normal_at_p = GradientAt(ScalarFunction, p);
        const auto LocalCurvatureRadius =
            min(
                RadiusOfCurvature(p, normal_at_p, u, normal_at_u),
                RadiusOfCurvature(p, normal_at_p, v, normal_at_v));
        const auto L = LocalCurvatureRadius / rho;

        const auto dHalf = uvd / 2.0;
        const auto IsoscelesHeight = sqrt(L * L - dHalf * dHalf);

        p = m + perpv * IsoscelesHeight;
        const auto dpu3 = (p - u).norm();
        p = ProjectPointToSurface(ScalarFunction, isoLevel, p);
        if (p < BoundingBox[0] || BoundingBox[1] < p)
        {
            spdlog::warn("Point {} falls out of bounding box", p);
            continue;
        }

        const auto dpu = (p - u).norm();
        assert_near(dpu, uvd, 1e-2);
        const auto dpv = (p - v).norm();
        assert_near(dpv, uvd, 1e-2);

        const auto angleIsGood = AngleIsGEThan(p - u, uv, M_PI / 4.0);
        spdlog::info("Angle criteria: {}", angleIsGood);
        const auto checkResult = CheckNeighbourTriangles(state, {u, v, p}, *edge);
        const auto distanceIsGood = !holds_alternative<TrianglesTooNear>(checkResult);
        spdlog::info("Distance criteria: {}", distanceIsGood);

        if (angleIsGood && distanceIsGood)
        {
            success = true;
            double longestSide;
            OpenMesh::SmartFaceHandle newFaceHandle;
            if (holds_alternative<AddNewVertex>(checkResult))
            {
                const auto outerHalfEdge = state.mesh.opposite_halfedge_handle(innerFaceHalfEdge);
                const auto newVertexhandle = state.mesh.add_vertex(p);
                assert(newVertexhandle != OpenMesh::PolyConnectivity::InvalidVertexHandle);
                newFaceHandle = state.mesh.add_face(
                    state.mesh.from_vertex_handle(outerHalfEdge),
                    state.mesh.to_vertex_handle(outerHalfEdge),
                    newVertexhandle);
                state.kdtree.buildIndex();
                longestSide = get<AddNewVertex>(checkResult).LongestSide;
                spdlog::info("Adding new vertex {}", newVertexhandle.idx());
            }
            else
            {
                const auto &useExistingVertices = get<UseExistingVertices>(checkResult);
                longestSide = useExistingVertices.LongestSide;
                state.mesh.data(useExistingVertices.oppositeEdge).GrownAlready = true;
                newFaceHandle = state.mesh.add_face(useExistingVertices.vertices, 3);
                spdlog::info("Reusing existing vertices {},{},{}. Edge {} will not be grown",
                             useExistingVertices.vertices[0].idx(),
                             useExistingVertices.vertices[1].idx(),
                             useExistingVertices.vertices[2].idx(),
                             useExistingVertices.oppositeEdge.idx());
            }
            assert(newFaceHandle != OpenMesh::PolyConnectivity::InvalidFaceHandle);

            if (state.longestSidedTriangle.longestSide < longestSide)
            {
                state.longestSidedTriangle.longestSide = longestSide;
                state.longestSidedTriangle.FaceHandle = newFaceHandle;
            }

            EarCutting(state);
        }
    }

    return success;
}

Geometry::TriMesh
CurvatureDependentTriangulation::Tessellate(
    function<autodiff::dual(
        const autodiff::dual &x,
        const autodiff::dual &y,
        const autodiff::dual &z)>
        ScalarFunction,
    const array<ImplicitTriangulation::Vector3D, 2> &BoundingBox,
    const double IsoLevel,
    const double Rho, // Ratio of triangle edge length to local radius of curvature
    const ImplicitTriangulation::Vector3D &InnerPoint,
    const ImplicitTriangulation::Vector3D &OuterPoint)
{
    spdlog::set_level(spdlog::level::info);
    spdlog::info("Starting triangulation");

    GrowingMesh Mesh;
    const auto NullOrderScalarFunction = ConvertToNullOrder(ScalarFunction);
    const auto SurfacePoint = Bisection(IsoLevel, NullOrderScalarFunction, InnerPoint, OuterPoint);

    // 1. Kezdő háromszög meghatározása
    // Tudni kell hozzá: egy x felületi pontot
    // Kapnunk: 3 csúcsot, amik egy (kb) egyenlő oldalú háromszöget alkotnak, aminek az oldalainak hossza
    // RadiusOfCurvature(x) / Rho

    const auto Gradient = GradientAt(ScalarFunction, SurfacePoint).normalized();
    const auto Tangent = GetPerpendicular(Gradient) * StartingTriangleStepScale;

    auto MinimalCurvateRadius = numeric_limits<double>::max();
    for (int step = 0; step < 12; ++step)
    {
        const auto CurrentTangent = Rotate(Gradient, static_cast<double>(step) * M_PI / 6.0, Tangent);
        const auto OtherSurfacePoint = ProjectPointToSurface(ScalarFunction, IsoLevel, SurfacePoint + CurrentTangent);
        assert_near(ScalarFunction(OtherSurfacePoint[0], OtherSurfacePoint[1], OtherSurfacePoint[2]).val, IsoLevel, 1e-5);
        MinimalCurvateRadius = min(MinimalCurvateRadius, RadiusOfCurvature(
                                                             SurfacePoint,
                                                             GradientAt(ScalarFunction, SurfacePoint),
                                                             OtherSurfacePoint,
                                                             GradientAt(ScalarFunction, OtherSurfacePoint)));
    }
    assert(MinimalCurvateRadius != numeric_limits<double>::max());

    const auto StartingTriangleSide = MinimalCurvateRadius / Rho;

    const auto &A = SurfacePoint;
    const auto B = ProjectPointToSurface(ScalarFunction, IsoLevel, SurfacePoint + Tangent * StartingTriangleSide);
    const auto Axis = A - B;
    const auto AxisLen = Axis.norm();
    // assert_near(StartingTriangleSide, AxisLen, 1e-3);
    const auto MidPoint = (A + B) / 2.0;
    const auto TriangleTangent = GradientAt(ScalarFunction, MidPoint).normalized() * Axis.norm() * l;
    auto BestError = numeric_limits<double>::max();
    ImplicitTriangulation::Vector3D C;
    // Try to find a C point so that A,B,C forms an equilateral triangle
    for (int step = 0; step < 12; ++step)
    {
        auto CurrentTangent = Rotate(Axis, static_cast<double>(step) * M_PI / 6.0, TriangleTangent);
        auto OtherSurfacePoint = MidPoint + CurrentTangent;
        OtherSurfacePoint = ProjectPointToSurface(ScalarFunction, IsoLevel, OtherSurfacePoint);
        const auto ErrorA = (A - OtherSurfacePoint).norm() - AxisLen;
        const auto ErrorB = (B - OtherSurfacePoint).norm() - AxisLen;
        const auto Error = ErrorA * ErrorA + ErrorB * ErrorB;
        if (Error < BestError)
        {
            BestError = Error;
            C = OtherSurfacePoint;
        }
    }

    assert(BoundingBox[0] < A && A < BoundingBox[1]);
    assert(BoundingBox[0] < B && B < BoundingBox[1]);
    assert(BoundingBox[0] < C && C < BoundingBox[1]);
    assert(IsValid(A));
    assert(IsValid(B));
    assert(IsValid(C));
    const auto dab = (A - B).norm();
    const auto dbc = (B - C).norm();
    const auto dac = (A - C).norm();
    spdlog::info("Starting triangle computed.\nPoints: ({},{},{})\nSides: ({:.5f},{:.5f},{:.5f})", A, B, C, dab, dbc, dac);
    assert(BestError < 0.5e-2);
    assert_near(dab, AxisLen, 1e-3);
    // assert_near(dbc, AxisLen, 1e-3);
    // assert_near(dac, AxisLen, 1e-3);

    // 2. Kiindulási állapot létrehozása
    const auto FirstFace = Mesh.add_face(Mesh.add_vertex(A), Mesh.add_vertex(B), Mesh.add_vertex(C));
    assert(FirstFace != OpenMesh::PolyConnectivity::InvalidFaceHandle);

    PointCloud<double, ImplicitTriangulation::Vector3D> pointCloud{Mesh};
    KDTree Kdtree{3, pointCloud};
    Kdtree.buildIndex();

    GrowingState growingState{Mesh, Kdtree, LongestSidedTriangleInfo()};

    growingState.longestSidedTriangle.FaceHandle = FirstFace;
    growingState.longestSidedTriangle.longestSide = LongestSide(A, B, C);

    bool OperationTookPlace = true;
    while (OperationTookPlace)
    {
        OperationTookPlace = false;

        OperationTookPlace = IsoscelesTriangleGrowing(ScalarFunction, IsoLevel, Rho, growingState, BoundingBox) || OperationTookPlace;
        // OperationTookPlace = EarCutting(growingState) || OperationTookPlace;
    }

    // 4. Kitöltési folyamat:
    // A növelési folyamt végén lesznek olyan régiók, amik még nincsenek háromszögelve. Végig kell menni a háromszögelt részek szélein lévő csúcsokon,
    // és meg kell határozni, hogy a széleken lévő csúcsok közül melyik van legközelebb az aktuális csúcshoz

    spdlog::info("Triangulation finished");
    return GrowingMeshToTrimesh(Mesh);
}