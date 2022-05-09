#pragma once

#include <nanoflann.hpp>

#include "glmMeshAdaptor.hpp"

template <typename mesh_t, typename num_t, typename point_t>
class GlmMeshKdTreeAdaptor
{
private:
    const mesh_t& mesh;
   
public:
    explicit GlmMeshKdTreeAdaptor(const mesh_t& mesh) : mesh(mesh) {}

    inline size_t kdtree_get_point_count() const { return mesh.n_vertices(); }

    inline num_t kdtree_get_pt(const unsigned int idx, const typename point_t::length_type dim) const
    {
        const auto points = mesh.points();
        return points[idx][dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

using GlmTriMeshKdTreeAdaptor = GlmMeshKdTreeAdaptor<GlmTriMesh, double, glm::dvec3>;
using GlmTriMeshKdTree = nanoflann::KDTreeSingleIndexAdaptor
    <nanoflann::L2_Simple_Adaptor<double, GlmTriMeshKdTreeAdaptor>, GlmTriMeshKdTreeAdaptor, 3, unsigned int>;

using GlmPolyMeshKdTreeAdaptor = GlmMeshKdTreeAdaptor<GlmPolyMesh, double, glm::dvec3>;
using GlmPolyMeshKdTree = nanoflann::KDTreeSingleIndexAdaptor
    <nanoflann::L2_Simple_Adaptor<double, GlmPolyMeshKdTreeAdaptor>, GlmPolyMeshKdTreeAdaptor, 3, unsigned int>;