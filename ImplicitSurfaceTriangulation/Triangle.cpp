#include "Triangle.hpp"
#include "VectorMath.hpp"

#define FMT_HEADER_ONLY
#include <fmt/format.h>
#include <array>
#include <iostream>

std::tuple<double, double, double> Triangle::GetSideLengths() const
{
    return { glm::distance(a,b), glm::distance(b, c), glm::distance(a, c) };
}

double Triangle::GetLongestSide() const
{
    auto sides = GetSideLengths();
    return std::max({std::get<0>(sides), std::get<1>(sides), std::get<2>(sides) });
}

glm::dvec3 Triangle::GetNormal() const
{
    return glm::normalize(glm::cross(glm::normalize(b-a), glm::normalize(c-a)));
}

glm::dvec3 Triangle::GetAltitude(int vertex) const
{
    std::array<std::reference_wrapper<const glm::dvec3>, 3> points = { a,b,c };
    auto projected = points[vertex%3].get() - points[(vertex - 1) % 3].get();
    auto target = points[vertex % 3].get() - points[(vertex + 1) % 3].get();
    return projected - ::Project(projected, target);
}

glm::dvec3 Triangle::GetCentroid() const
{
    return (a+b+c)/3.;
}

glm::dvec3 Triangle::ProjectPoint(const glm::dvec3& p) const
{
    const glm::dvec3& q1 = a, q2 = b, q3 = c;
    // As in Schneider, Eberly: Geometric Tools for Computer Graphics, Morgan Kaufmann, 2003.
    // Section 10.3.2, pp. 376-382 (with my corrections)
    const glm::dvec3& P = p, & B = q1;
    glm::dvec3 E0 = q2 - q1, E1 = q3 - q1, D = B - P;
    double a = glm::dot(E0, E0), b = glm::dot(E0, E1), c = glm::dot(E1,E1), d = glm::dot(E0,D), e = glm::dot(E1, D);
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
                    s = 0.0f;
                    t = (-e >= c ? 1.0f : -e / c);
                }
                else if (d < 0)
                {
                    t = 0.0;
                    s = (-d >= a ? 1.0f : -d / a);
                }
                else
                {
                    s = 0.0f;
                    t = 0.0f;
                }
            }
            else
            {
                // Region 3
                s = 0.0f;
                t = (e >= 0.0f ? 0.0f : (-e >= c ? 1.0f : -e / c));
            }
        }
        else if (t < 0)
        {
            // Region 5
            t = 0.0;
            s = (d >= 0.0f ? 0.0f : (-d >= a ? 1.0f : -d / a));
        }
        else
        {
            // Region 0
            double invDet = 1.0f / det;
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
                s = (numer >= denom ? 1.0f : numer / denom);
                t = 1.0f - s;
            }
            else
            {
                s = 0.0f;
                t = (tmp1 <= 0.0f ? 1.0f : (e >= 0.0f ? 0.0f : -e / c));
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
                t = (numer >= denom ? 1.0f : numer / denom);
                s = 1.0f - t;
            }
            else
            {
                t = 0.0;
                s = (tmp1 <= 0.0f ? 1.0f : (d >= 0.0f ? 0.0f : -d / a));
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
                s = (numer >= denom ? 1.0f : numer / denom);
            }
            t = 1.0f - s;
        }
    }
    return B + E0 * s + E1 * t;
}

double Triangle::GetDistanceFrom(const glm::dvec3& point) const
{
    const glm::dvec3 c = ProjectPoint(point);
    const double q = glm::distance(c, point);
    return q;
}

glm::dvec3 Triangle::getItp(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& normalizedTangent, double equalSideLength)
{
    const glm::dvec3 midPoint = (a + b) / 2.;
    const double midDistance = glm::distance(midPoint, b);
    if (midDistance >= equalSideLength) {
        std::cerr << "equalSideLength is too small\n";
        equalSideLength = midDistance;
    }

    const double height = sqrt(equalSideLength * equalSideLength - midDistance * midDistance);
    return midPoint + height * normalizedTangent;
}

glm::dvec3 Triangle::getEtp(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& normalizedTangent)
{
    return getItp(a, b, normalizedTangent, glm::distance(a, b));
}

template <>
struct fmt::formatter<glm::dvec3>
{
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
    {
        return ctx.end();
    }

    template <typename FormatContext>
    auto format(const glm::dvec3& p, FormatContext& ctx) -> decltype(ctx.out())
    {
        return format_to(ctx.out(),
            "({:.5f},{:.5f},{:.5f})",
            p[0], p[1], p[2]);
    }
};


std::string Triangle::to_string() const
{
    auto sides = GetSideLengths();
    return fmt::format("Points:\n{}\n{}\n{}\nSides: {}, {}, {}", a, b, c, std::get<0>(sides), std::get<1>(sides), std::get<2>(sides));
}
