#pragma once

#include <functional>
#include <geometry.hh>
#include <autodiff/forward/dual.hpp>

namespace CurvatureDependentTriangulation // Curvature Dependent Triangulation
{
    Geometry::Vector3D
    ProjectPointToSurface(
        std::function<autodiff::dual(
            const autodiff::dual &x,
            const autodiff::dual &y,
            const autodiff::dual &z)>
            ScalarFunction,
        const double IsoLevel,
        const Geometry::Vector3D &Point);

    Geometry::TriMesh
    Tessellate(
        std::function<autodiff::dual(
            const autodiff::dual &x,
            const autodiff::dual &y,
            const autodiff::dual &z)>
            ScalarFunction,
        const std::array<Geometry::Vector3D, 2> &BoundingBox,
        const double IsoLevel,
        const double Rho, // Ratio of triangle edge length to local radius of curvature
        const Geometry::Vector3D &InnerPoint,
        const Geometry::Vector3D &OuterPoint);
}