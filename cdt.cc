#include <limits>
#include <nanoflann.hpp>
#include <algorithm>
#include <numeric>
#include <map>
#include <set>

#include "cdt.hh"

using namespace std;
using namespace Geometry;
using namespace autodiff;
using namespace nanoflann;

constexpr double eps = 1e-8;
constexpr int BisectionIterations = 10000;
constexpr int ProjectionIterations = 10000;
constexpr double ProjectionStepScale = 0.01;
constexpr double StartingTriangleStepScale = 0.07; // 0.2, 0.6 jó

static const double l = sqrt(3.0) / 2.0;

static Vector3D
GetPerpendicular(const Vector3D &vec)
{
    auto perp = vec ^ Vector3D(1, 0, 0);
    auto n = perp.norm();

    if (n == 0)
    {
        perp = vec ^ Vector3D(0, 1, 0);
        n = perp.norm();
    }

    if (n != 0.0)
    {
        perp = perp * 1.0 / n;
    }

    return perp * vec.norm();
}

template <typename num_t, typename point_t>
struct PointCloud
{
    vector<point_t> &pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        return pts[idx][dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
};

Point3D
projectToTriangle(const Point3D &p, const TriMesh::Triangle &tri, const PointVector &points_)
{
    const Point3D &q1 = points_[tri[0]], &q2 = points_[tri[1]], &q3 = points_[tri[2]];
    // As in Schneider, Eberly: Geometric Tools for Computer Graphics, Morgan Kaufmann, 2003.
    // Section 10.3.2, pp. 376-382 (with my corrections)
    const Point3D &P = p, &B = q1;
    Vector3D E0 = q2 - q1, E1 = q3 - q1, D = B - P;
    double a = E0 * E0, b = E0 * E1, c = E1 * E1, d = E0 * D, e = E1 * D;
    double det = a * c - b * b, s = b * e - c * d, t = b * d - a * e;
    if (s + t <= det)
    {
        if (s < 0)
        {
            if (t < 0)
            {
                // Region 4
                if (e < 0)
                {
                    s = 0.0;
                    t = (-e >= c ? 1.0 : -e / c);
                }
                else if (d < 0)
                {
                    t = 0.0;
                    s = (-d >= a ? 1.0 : -d / a);
                }
                else
                {
                    s = 0.0;
                    t = 0.0;
                }
            }
            else
            {
                // Region 3
                s = 0.0;
                t = (e >= 0.0 ? 0.0 : (-e >= c ? 1.0 : -e / c));
            }
        }
        else if (t < 0)
        {
            // Region 5
            t = 0.0;
            s = (d >= 0.0 ? 0.0 : (-d >= a ? 1.0 : -d / a));
        }
        else
        {
            // Region 0
            double invDet = 1.0 / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if (s < 0)
        {
            // Region 2
            double tmp0 = b + d, tmp1 = c + e;
            if (tmp1 > tmp0)
            {
                double numer = tmp1 - tmp0;
                double denom = a - 2 * b + c;
                s = (numer >= denom ? 1.0 : numer / denom);
                t = 1.0 - s;
            }
            else
            {
                s = 0.0;
                t = (tmp1 <= 0.0 ? 1.0 : (e >= 0.0 ? 0.0 : -e / c));
            }
        }
        else if (t < 0)
        {
            // Region 6
            double tmp0 = b + e, tmp1 = a + d;
            if (tmp1 > tmp0)
            {
                double numer = tmp1 - tmp0;
                double denom = c - 2 * b + a;
                t = (numer >= denom ? 1.0 : numer / denom);
                s = 1.0 - t;
            }
            else
            {
                t = 0.0;
                s = (tmp1 <= 0.0 ? 1.0 : (d >= 0.0 ? 0.0 : -d / a));
            }
        }
        else
        {
            // Region 1
            double numer = c + e - b - d;
            if (numer <= 0)
            {
                s = 0;
            }
            else
            {
                double denom = a - 2 * b + c;
                s = (numer >= denom ? 1.0 : numer / denom);
            }
            t = 1.0 - s;
        }
    }
    return B + E0 * s + E1 * t;
}

using KDTree = KDTreeSingleIndexAdaptor<
    L2_Simple_Adaptor<double, PointCloud<double, Point3D>>,
    PointCloud<double, Point3D>,
    3>;

struct Edge
{
    size_t u;
    size_t v;
    size_t q;
    bool TriedTriangleGrowing;
};

using Edges = map<size_t, vector<Edge>>;
using Triangles = vector<TriMesh::Triangle>;

struct LongestSidedTriangleInfo
{
    size_t index;
    double longestSide = numeric_limits<double>::min();
};

struct GrowingState
{
    PointVector &points;
    Edges &edges;
    Triangles &triangles;
    KDTree &kdtree;
    LongestSidedTriangleInfo longestSidedTriangle;
};

static Vector3D
Bisection(double isolevel, function<double(Vector3D)> scalarFunc, Vector3D p1, Vector3D p2)
{
    double valp1 = scalarFunc(p1) - isolevel;
    if (abs(valp1) < eps)
        return p1;
    if (abs(scalarFunc(p2) - isolevel) < eps)
        return p2;
    auto p = (p1 + p2) / 2;
    int i = 0;
    double valp = scalarFunc(p) - isolevel;
    while (abs(valp) > eps && i < BisectionIterations)
    {
        if (valp * valp1 < 0.0)
        {
            p2 = p;
        }
        else
        {
            p1 = p;
            valp1 = valp;
        }
        p = (p1 + p2) / 2;
        valp = scalarFunc(p) - isolevel;
        ++i;
    }
    return p;
}

static double
RadiusOfCurvature(const Vector3D &x, const Vector3D &y)
{
    auto d = (x - y).norm();
    return d / sqrt(2 - 2 * (x * y));
}

static bool
OppositeSigns(double left, double right)
{
    return left >= 0 && right <= 0 || left <= 0 && right >= 0;
}

static function<double(const Vector3D &)>
ConvertToNullOrder(function<dual(
                       const dual &x,
                       const dual &y,
                       const dual &z)>
                       ScalarFunction)
{
    return [ScalarFunction](const Vector3D &v) -> double
    {
        return ScalarFunction(v[0], v[1], v[2]).val;
    };
}

// Pont (jelölje v) vetítése a felületre:
// Feltétel: v közel helyezkedik el a felülethez
// Meghatározunk egy v' pontot, ami a felület túlsó oldalára esik, és biszekcióval kaphatunk ezután egy
// olyan pontot, ami a felületre esik
// Az n = - f(v) * grad f(v) / norm( f(v) * grad f(v) ) normálvektor a v-ből a felület felé mutat.
// Kérdés: mi értelme van f(v)-vel szorozni grad f(v)-t, ha úgyis normalizálnuk?
// Valószínűleg azért, hogy a vektor a jó irányba nézzen.
// Ezután növekvő távolságokra egymástól mintavételezzük a sugarat addig, amíg egy olyan v' pontot nem találnuk,
// hogy f(v) előjele az ellentettje f(v') előjelének
// Kérdés: milyen függvénnyel növekedjen a mintavételezési távolság? lineáris? exponenciális?

static Vector3D
GradientAt(
    function<dual(
        const dual &x,
        const dual &y,
        const dual &z)>
        ScalarFunction,
    dual &x, dual &y, dual &z)
{
    const double ux = derivative(ScalarFunction, wrt(x), at(x, y, z));
    const double uy = derivative(ScalarFunction, wrt(y), at(x, y, z));
    const double uz = derivative(ScalarFunction, wrt(z), at(x, y, z));
    const Vector3D Gradient{ux, uy, uz};
    return Gradient;
}

static Vector3D
GradientAt(
    function<dual(
        const dual &x,
        const dual &y,
        const dual &z)>
        ScalarFunction,
    const Vector3D &p)
{
    dual x = p[0], y = p[1], z = p[2];
    return GradientAt(ScalarFunction, x, y, z);
}

Vector3D
CurvatureDependentTriangulation::ProjectPointToSurface(
    function<dual(
        const dual &x,
        const dual &y,
        const dual &z)>
        ScalarFunction,
    const double IsoLevel,
    const Vector3D &Point)
{
    dual x = Point[0], y = Point[1], z = Point[2], u = ScalarFunction(x, y, z);

    if (abs(u.val - IsoLevel) < eps)
        return Point;

    const Vector3D Gradient = GradientAt(ScalarFunction, x, y, z);
    const Vector3D DirectionToSurface = -(Gradient * (ScalarFunction(x, y, z).val - IsoLevel)).normalized();

    int step = 1;
    Vector3D PointOnTheOtherSide;
    double k;
    const auto NullOrderScalarFunction = ConvertToNullOrder(ScalarFunction);
    do
    {
        PointOnTheOtherSide = Point + DirectionToSurface * ProjectionStepScale * step;
        k = NullOrderScalarFunction(PointOnTheOtherSide);
        ++step;
    } while (step <= ProjectionIterations && !OppositeSigns(u.val - IsoLevel, k - IsoLevel));

    if (step == ProjectionIterations)
        return Point;
    else
        return Bisection(IsoLevel, NullOrderScalarFunction, Point, PointOnTheOtherSide);
}

static double
Degrees(const Vector3D &x, const Vector3D &y)
{
    return acos(x.normalized() * y.normalized()) * 180.0 / M_PI;
}

// Vectors must be normalized
static bool
AngleIsGEThan(const Vector3D &x, const Vector3D &y, const double angle)
{
    const auto xyAngle = acos(x.normalized() * y.normalized());
    return xyAngle >= angle;
}

struct PointCompare
{
    bool operator()(const Point3D &lhs, const Point3D &rhs) const
    {
        return std::tuple<double, double, double>(lhs[0], lhs[1], lhs[2]) < std::tuple<double, double, double>(rhs[0], rhs[1], rhs[2]);
    }
};

static bool
NeighbourTrianglesAreFarEnough(const GrowingState &state, array<Point3D, 3> newTriangle, double &longestSide)
{
    const auto &LongestSidedTriangle = state.triangles.at(state.longestSidedTriangle.index);

    const auto c = projectToTriangle(newTriangle.back(), LongestSidedTriangle, state.points);
    const auto q = (newTriangle.back() - c).norm();

    longestSide = max({(newTriangle[0] - newTriangle[1]).norm(),
                       (newTriangle[0] - newTriangle[2]).norm(),
                       (newTriangle[1] - newTriangle[2]).norm(),
                       state.longestSidedTriangle.longestSide});

    const auto t = 2.0 * longestSide / 3.0 + q;
    const auto v = state.longestSidedTriangle.longestSide / 2.0;
    const auto r = sqrt(t * t + v * v);

    vector<pair<size_t, double>> ret_matches;
    SearchParams params;
    params.sorted = true;

    const auto Centroid = accumulate(newTriangle.begin(), newTriangle.end(), Point3D(0, 0, 0)) / 3.0;

    const auto matchCount = state.kdtree.radiusSearch(Centroid.data(), r, ret_matches, params);

    for (const auto &match : ret_matches)
    {
        const auto &edges_for_vertex = state.edges[match.first];

        for (const auto &edge : edges_for_vertex)
        {

            std::set<Point3D, PointCompare> EdgeIndices;
            EdgeIndices.insert(state.points[edge.u]);
            EdgeIndices.insert(state.points[edge.v]);
            EdgeIndices.insert(state.points[edge.q]);

            size_t CommonPoints = EdgeIndices.count(newTriangle[0]) + EdgeIndices.count(newTriangle[1]) + EdgeIndices.count(newTriangle[2]);

            if (CommonPoints == 2)
            {
                continue;
            }

            const auto &A = state.points[edge.u];
            const auto &B = state.points[edge.v];
            const auto &C = state.points[edge.q];

            const double longestSide =
                max({(newTriangle[0] - newTriangle[1]).norm(),
                     (newTriangle[0] - newTriangle[2]).norm(),
                     (newTriangle[1] - newTriangle[2]).norm(),
                     (A - B).norm(), (A - C).norm(), (B - C).norm()});

            const auto c = projectToTriangle(newTriangle.back(), {edge.u, edge.v, edge.q}, state.points);
            const auto q = (newTriangle.back() - c).norm();

            if (q < longestSide / 2.0)
            {
                return false;
            }
        }
    }

    return true;
}

static void
AddEdge(Edges &edges, const Edge &edge)
{
    edges[edge.u].push_back(edge);
    edges[edge.v].push_back(edge);
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
    GrowingState &state)
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

    for (auto edge_mapping = state.edges.begin(); edge_mapping != state.edges.end(); ++edge_mapping)
    {
        auto &edges = edge_mapping->second;
        for (auto edge = edges.begin(); edge != edges.end(); ++edge)
        {
            if (edge_mapping->first == edge->u && !edge->TriedTriangleGrowing)
            {

                edge->TriedTriangleGrowing = true;

                const auto u = state.points.at(edge->u);
                const auto v = state.points.at(edge->v);
                const auto q = state.points.at(edge->q);

                const auto uv = v - u;
                const auto d = uv.norm();
                const auto uvn = uv.normalized();
                const auto height = l * d;

                const auto m = (u + v) / 2.0;

                const auto qu = u - q;
                const auto qv = v - q;
                const auto normal = (qv ^ qu).normalized();
                const auto perpv = (normal ^ uv).normalized() * height;

                Vector3D p = CurvatureDependentTriangulation::ProjectPointToSurface(ScalarFunction, isoLevel, m + perpv);

                const auto LocalCurvatureRadius = min(RadiusOfCurvature(p, u), RadiusOfCurvature(p, v));
                const auto L = LocalCurvatureRadius * rho;

                const auto dHalf = d / 2.0;
                const auto IsoscelesHeight = sqrt(L * L - dHalf * dHalf);

                p = m + perpv * IsoscelesHeight;
                p = CurvatureDependentTriangulation::ProjectPointToSurface(ScalarFunction, isoLevel, p);

                double longestSide;
                const bool angleIsGood = AngleIsGEThan(p - u, uv, M_PI / 4.0);
                const bool distanceIsGood = NeighbourTrianglesAreFarEnough(state, {u, v, p}, longestSide);
                const bool success = angleIsGood && distanceIsGood;

                if (success)
                {
                    state.points.push_back(p);
                    state.kdtree.buildIndex();
                    const auto index = state.points.size() - 1;
                    state.triangles.push_back({edge->u, edge->v, index});

                    {
                        const size_t current_edge_index = edge - edges.begin();
                        size_t edge_map_key = edge_mapping->first;
                        Edge e = *edge;
                        AddEdge(state.edges, {e.u, index, e.v, false});
                        AddEdge(state.edges, {e.v, index, e.u, false});

                        // Set invalidated iterators
                        edge_mapping = state.edges.find(edge_map_key);
                        edge = std::next(edge_mapping->second.begin(), current_edge_index);
                    }

                    if (state.longestSidedTriangle.longestSide < longestSide)
                    {
                        state.longestSidedTriangle.longestSide = longestSide;
                        state.longestSidedTriangle.index = state.triangles.size() - 1;
                    }

                    //if (state.points.size() >= 9)
                    //    return false;
                }
            }
        }
    }

    return success;
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

    vector<Edge> NewEdges;
    for (auto &edge_mapping : state.edges)
    {
        auto &edges = edge_mapping.second;
        for (auto e_it = edges.begin(); e_it != edges.end(); ++e_it)
        {
            for (auto a_it = std::next(e_it); a_it != edges.end(); ++a_it)
            {
                Edge e1 = *e_it;
                Edge e2 = *a_it;

                if (e1.v == e2.u || e1.u == e2.v)
                {
                    std::swap(e2.u, e2.v);
                }

                const Vector3D &A = state.points.at(e1.u);
                const Vector3D &B = state.points.at(e1.v);

                const Vector3D &C = state.points.at(e2.u);
                const Vector3D &D = state.points.at(e2.v);

                Vector3D DV1 = B - A;
                Vector3D DV2 = D - C;
                if (e1.v == e2.v)
                {
                    DV1 *= -1.0;
                    DV2 *= -1.0;
                }

                const bool needsCut = AngleIsGEThan(DV1, DV2, M_PI * 110.0 / 180.0);

                if (needsCut)
                {
                    NewEdges.push_back(Edge{e1.v, e2.v, e1.q, false});

                    const size_t indexA = a_it - edges.begin();

                    e_it = edges.erase(e_it);
                    a_it = std::next(e_it, indexA);

                    state.kdtree.buildIndex();
                }

                OperationTookPlace = OperationTookPlace || needsCut;
            }
        }
    }

    for (const auto &edge : NewEdges)
    {
        AddEdge(state.edges, edge);
    }

    return OperationTookPlace;
}

Geometry::TriMesh
CurvatureDependentTriangulation::Tessellate(
    function<autodiff::dual(
        const autodiff::dual &x,
        const autodiff::dual &y,
        const autodiff::dual &z)>
        ScalarFunction,
    const array<Geometry::Vector3D, 2> &BoundingBox,
    const double IsoLevel,
    const double Rho, // Ratio of triangle edge length to local radius of curvature
    const Geometry::Vector3D &InnerPoint,
    const Geometry::Vector3D &OuterPoint)
{
    TriMesh Mesh;
    PointVector Points;

    const auto NullOrderScalarFunction = ConvertToNullOrder(ScalarFunction);
    const auto SurfacePoint = Bisection(IsoLevel, NullOrderScalarFunction, InnerPoint, OuterPoint);

    // 1. Kezdő háromszög meghatározása
    // Tudni kell hozzá: egy x felületi pontot
    // Kapnunk: 3 csúcsot, amik egy (kb) egyenlő oldalú háromszöget alkotnak, aminek az oldalainak hossza
    // RadiusOfCurvature(x) / Rho

    const auto Gradient = GradientAt(ScalarFunction, SurfacePoint).normalized() * StartingTriangleStepScale;
    const auto Tangent = GetPerpendicular(Gradient);

    auto MinimalCurvateRadius = numeric_limits<double>::max();
    for (int step = 0; step < 12; ++step)
    {
        auto CurrentTangent = Matrix3x3::rotation(Gradient, static_cast<double>(step) * M_PI / 6.0) * Tangent;
        auto OtherSurfacePoint = ProjectPointToSurface(ScalarFunction, IsoLevel, SurfacePoint + CurrentTangent);
        MinimalCurvateRadius = min(MinimalCurvateRadius, RadiusOfCurvature(SurfacePoint, OtherSurfacePoint));
    }

    const auto StartingTriangleSide = MinimalCurvateRadius / Rho;

    const auto &A = SurfacePoint;
    const auto B = ProjectPointToSurface(ScalarFunction, IsoLevel, SurfacePoint + Tangent * StartingTriangleSide);

    const auto Axis = A - B;
    const auto AxisLen = Axis.norm();
    const auto MidPoint = (A + B) / 2.0;
    const auto TriangleTangent = GradientAt(ScalarFunction, MidPoint).normalized() * Axis.norm() * l;
    auto BestError = numeric_limits<double>::max();
    Vector3D C;
    // Try to find a C point so that A,B,C forms an equilateral triangle
    for (int step = 0; step < 12; ++step)
    {
        auto CurrentTangent = Matrix3x3::rotation(Axis.normalized(), static_cast<double>(step) * M_PI / 6.0) * TriangleTangent;
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

    // 2. Aktív éllista létrehozása
    Points.push_back(A);
    Points.push_back(B);
    Points.push_back(C);

    Triangles Triangles;
    Triangles.push_back({0, 1, 2});

    Edges Edges;
    AddEdge(Edges, {0, 1, 2, false});
    AddEdge(Edges, {1, 2, 0, false});
    AddEdge(Edges, {0, 2, 1, false});

    PointCloud<double, Point3D> pointCloud{Points};
    KDTree Kdtree{3, pointCloud};
    Kdtree.buildIndex();

    GrowingState growingState{Points, Edges, Triangles, Kdtree};

    growingState.longestSidedTriangle.index = 0;
    growingState.longestSidedTriangle.longestSide = max({(A - B).norm(), (A - C).norm(), (B - C).norm()});

    bool OperationTookPlace = true;
    while (OperationTookPlace)
    {
        OperationTookPlace = false;

        OperationTookPlace = IsoscelesTriangleGrowing(ScalarFunction, IsoLevel, Rho, growingState) || OperationTookPlace;
        //OperationTookPlace = EarCutting(growingState) || OperationTookPlace;
    }

    // 4. Kitöltési folyamat:
    // A növelési folyamt végén lesznek olyan régiók, amik még nincsenek háromszögelve. Végig kell menni a háromszögelt részek szélein lévő csúcsokon,
    // és meg kell határozni, hogy a széleken lévő csúcsok közül melyik van legközelebb az aktuális csúcshoz

    Mesh.setPoints(Points);
    for (const auto &triangle : Triangles)
    {
        Mesh.addTriangle(triangle[0], triangle[1], triangle[2]);
    }

    return Mesh;
}