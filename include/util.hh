#pragma once

#include <functional>

#include <autodiff/forward/dual.hpp>
#include <geometry.hh>
#include <spdlog/fmt/bundled/format.h>
#include "types.hh"

ImplicitTriangulation::Vector3D Rotate(ImplicitTriangulation::Vector3D const &axis, double angle, ImplicitTriangulation::Vector3D const &vector);

ImplicitTriangulation::Vector3D GetPerpendicular(const ImplicitTriangulation::Vector3D &vec);

ImplicitTriangulation::Vector3D
ProjectToTriangle(const ImplicitTriangulation::Vector3D &p, const GrowingMesh::FaceHandle face, const GrowingMesh &mesh);

ImplicitTriangulation::Vector3D Bisection(
    double isolevel,
    std::function<double(ImplicitTriangulation::Vector3D)> scalarFunc,
    ImplicitTriangulation::Vector3D p1,
    ImplicitTriangulation::Vector3D p2);

double RadiusOfCurvature(const ImplicitTriangulation::Vector3D &x, const ImplicitTriangulation::Vector3D &nx,
                         const ImplicitTriangulation::Vector3D &y, const ImplicitTriangulation::Vector3D &ny);

bool OppositeSigns(double left, double right);

std::function<double(const ImplicitTriangulation::Vector3D &)>
ConvertToNullOrder(std::function<autodiff::dual(
                       const autodiff::dual &x,
                       const autodiff::dual &y,
                       const autodiff::dual &z)>
                       ScalarFunction);

ImplicitTriangulation::Vector3D
GradientAt(
    std::function<autodiff::dual(
        const autodiff::dual &x,
        const autodiff::dual &y,
        const autodiff::dual &z)>
        ScalarFunction,
    autodiff::dual &x, autodiff::dual &y, autodiff::dual &z);

ImplicitTriangulation::Vector3D
GradientAt(
    std::function<autodiff::dual(
        const autodiff::dual &x,
        const autodiff::dual &y,
        const autodiff::dual &z)>
        ScalarFunction,
    const ImplicitTriangulation::Vector3D &p);

ImplicitTriangulation::Vector3D
ProjectPointToSurface(
    std::function<autodiff::dual(
        const autodiff::dual &x,
        const autodiff::dual &y,
        const autodiff::dual &z)>
        ScalarFunction,
    const double IsoLevel,
    const ImplicitTriangulation::Vector3D &Point);

double Degrees(const ImplicitTriangulation::Vector3D &x, const ImplicitTriangulation::Vector3D &y);

bool AngleIsGEThan(const ImplicitTriangulation::Vector3D &x, const ImplicitTriangulation::Vector3D &y, const double angle);

double LongestSide(const ImplicitTriangulation::Vector3D &A, const ImplicitTriangulation::Vector3D &B, const ImplicitTriangulation::Vector3D &C);

template <>
struct fmt::formatter<ImplicitTriangulation::Vector3D>
{
    constexpr auto parse(format_parse_context &ctx) -> decltype(ctx.begin())
    {
        return ctx.end();
    }

    template <typename FormatContext>
    auto format(const ImplicitTriangulation::Vector3D &p, FormatContext &ctx) -> decltype(ctx.out())
    {
        return format_to(ctx.out(),
                         "({:.5f},{:.5f},{:.5f})",
                         p[0], p[1], p[2]);
    }
};

bool IsValid(const ImplicitTriangulation::Vector3D &vec);