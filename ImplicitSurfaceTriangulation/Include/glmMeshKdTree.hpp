#pragma once

#include <nanoflann.hpp>

#include "glmMeshAdaptor.hpp"

template <typename num_t, typename point_t>
class GlmMeshKdTreeAdaptor
{
private:
    const GlmMesh& mesh;
   
public:
    explicit GlmMeshKdTreeAdaptor(const GlmMesh& mesh) : mesh(mesh) {}

    inline size_t kdtree_get_point_count() const { return mesh.n_vertices(); }

    inline num_t kdtree_get_pt(const unsigned int idx, const typename point_t::length_type dim) const
    {
        const auto points = mesh.points();
        return points[idx][dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

using GlmVec3MeshKdTreeAdaptor = GlmMeshKdTreeAdaptor<double, glm::dvec3>;
using GlmMeshKdTree = nanoflann::KDTreeSingleIndexAdaptor
    <nanoflann::L2_Simple_Adaptor<double, GlmVec3MeshKdTreeAdaptor>, GlmVec3MeshKdTreeAdaptor, 3, unsigned int>;