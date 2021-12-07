#pragma once

#include <functional>
#include <geometry.hh>
#include <autodiff/forward/dual.hpp>

#include "types.hh"

namespace CurvatureDependentTriangulation // Curvature Dependent Triangulation
{
    Geometry::TriMesh
    Tessellate(
        std::function<autodiff::dual(
            const autodiff::dual &x,
            const autodiff::dual &y,
            const autodiff::dual &z)>
            ScalarFunction,
        const std::array<ImplicitTriangulation::Vector3D, 2> &BoundingBox,
        const double IsoLevel,
        const double Rho, // Ratio of triangle edge length to local radius of curvature
        const ImplicitTriangulation::Vector3D &InnerPoint,
        const ImplicitTriangulation::Vector3D &OuterPoint);
}