#include "util.hh"
#include "types.hh"
#include "constants.hh"

using namespace std;
using namespace autodiff;
using namespace ImplicitTriangulation;

Vector3D Rotate(Vector3D const &axis, double angle, Vector3D const &vector)
{
    const auto axis_normalized = axis.normalized();
    const auto _axis = Geometry::Vector3D(axis_normalized[0], axis_normalized[1], axis_normalized[2]);
    const auto rotated = Geometry::Matrix3x3::rotation(_axis, angle) * Geometry::Vector3D(vector[0], vector[1], vector[2]);
    return Vector3D(rotated[0], rotated[1], rotated[2]);
}

Vector3D
GetPerpendicular(const Vector3D &vec)
{
    auto perp = vec.cross(Vector3D(1, 0, 0));
    auto n = perp.norm();

    if (n == 0)
    {
        perp = vec.cross(Vector3D(0, 1, 0));
        n = perp.norm();
    }

    if (n != 0.0)
    {
        perp = perp * 1.0 / n;
    }

    return perp * vec.norm();
}

Vector3D
ProjectToTriangle(const Vector3D &p, const GrowingMesh::FaceHandle face, const GrowingMesh &mesh)
{
    const auto face_points = mesh.fv_range(face).to_vector();
    const Vector3D &q1 = mesh.point(face_points[0]),
                   &q2 = mesh.point(face_points[1]),
                   &q3 = mesh.point(face_points[2]);
    // As in Schneider, Eberly: Geometric Tools for Computer Graphics, Morgan Kaufmann, 2003.
    // Section 10.3.2, pp. 376-382 (with my corrections)
    const Vector3D &P = p, &B = q1;
    Vector3D E0 = q2 - q1, E1 = q3 - q1, D = B - P;
    double a = E0.dot(E0), b = E0.dot(E1), c = E1.dot(E1), d = E0.dot(D), e = E1.dot(D);
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

Vector3D Bisection(double isolevel, function<double(Vector3D)> scalarFunc, Vector3D p1, Vector3D p2)
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

double RadiusOfCurvature(const Vector3D &x, const Vector3D &nx, const Vector3D &y, const Vector3D &ny)
{
    const auto d = (x - y).norm();
    const auto theta = acos(nx.normalized().dot(ny.normalized()));
    const auto r = d / (2 * sin(theta / 2.0));
    return r;
}

bool OppositeSigns(double left, double right)
{
    return left > 0 && right < 0 || left < 0 && right > 0;
}

function<double(const Vector3D &)>
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

Vector3D
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

Vector3D
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

Vector3D
ProjectPointToSurface(
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

double
Degrees(const Vector3D &x, const Vector3D &y)
{
    return acos(x.normalized().dot(y.normalized())) * 180.0 / M_PI;
}

bool AngleIsGEThan(const Vector3D &x, const Vector3D &y, const double angle)
{
    const auto xyAngle = acos(x.normalized().dot(y.normalized()));
    return xyAngle >= angle;
}

double LongestSide(const ImplicitTriangulation::Vector3D &A, const ImplicitTriangulation::Vector3D &B, const ImplicitTriangulation::Vector3D &C)
{
    return max({(A - B).norm(), (B - C).norm(), (A - C).norm()});
}

bool IsValid(const Vector3D &vec)
{
    for (const auto &comp : vec)
        if (isnan(comp))
            return false;
    return true;
}