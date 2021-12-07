#pragma once

#include <limits>
#include <vector>
#include <map>
#include <nanoflann.hpp>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

struct LongestSidedTriangleInfo
{
    OpenMesh::SmartFaceHandle FaceHandle;
    double longestSide = std::numeric_limits<double>::min();
};

namespace ImplicitTriangulation
{
    using Vector3D = OpenMesh::Vec3d;
}

struct MeshTraits : public OpenMesh::DefaultTraits
{

    using Point = ImplicitTriangulation::Vector3D;
    using Normal = ImplicitTriangulation::Vector3D;
    EdgeTraits
    {
        bool GrownAlready = false;
    };
};

using GrowingMesh = OpenMesh::TriMesh_ArrayKernelT<MeshTraits>;

template <typename num_t, typename point_t>
struct PointCloud
{
    const GrowingMesh &mesh;

    inline size_t kdtree_get_point_count() const { return mesh.n_vertices(); }

    inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        const auto points = mesh.points();
        return points[idx][dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud<double, ImplicitTriangulation::Vector3D>>,
    PointCloud<double, ImplicitTriangulation::Vector3D>,
    3>;

struct GrowingState
{
    GrowingMesh &mesh;
    KDTree &kdtree;
    LongestSidedTriangleInfo longestSidedTriangle;
};
