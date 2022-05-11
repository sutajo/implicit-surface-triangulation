#pragma once

#include "ImplicitObject.hpp"
#include "glmMeshAdaptor.hpp"
#include "glmMeshKdTree.hpp"

namespace Implicit
{
	namespace Tessellation
	{
		class ClosestNeighbours
		{
		public:
			ClosestNeighbours(GlmPolyMesh& mesh, Object &object);

			/*
			Add a mesh vertex to the point list
			*/
			OpenMesh::VertexHandle AddVertex(OpenMesh::VertexHandle vertex, bool rebuildKdTree = false);

			/*
			Remove a mesh vertex from the point list
			*/
			void RemoveVertex(OpenMesh::VertexHandle vertex);

			/*
			Rebuild the KdTree
			*/
			void RebuildIndex();

			/*
			Compute the closest neighbour of the to vertex of the halfedge
			*/
			OpenMesh::VertexHandle ComputeClosestNeighbour(OpenMesh::SmartHalfedgeHandle heh);

			/*
			Update the closest neighbour of the to vertex of the halfedge
			*/
			void UpdateClosestNeighbour(OpenMesh::SmartHalfedgeHandle heh);

			/*
			Compute the closest neighbour of the every vertex of the face
			*/
			void UpdateClosestNeighbours(OpenMesh::SmartFaceHandle face);

			/*
			Moves the closest neighbour relationship data between faces
			*/
			void MoveFaceNeighbours(OpenMesh::FaceHandle from, OpenMesh::FaceHandle to);

			/*
			Is the to vertex of the halfedge convex?
			*/
			bool IsConvex(OpenMesh::SmartHalfedgeHandle toHalfedge) const;

			/*
			Is the vertex a neigbour of the "to" vertex of the halfedge?
			*/
			bool IsNeighbour(OpenMesh::SmartHalfedgeHandle toHalfedge, OpenMesh::VertexHandle vertex) const;

			/*
			Is the vertex part of a bridge?
			*/
			bool IsBridge(OpenMesh::SmartHalfedgeHandle toHalfedge) const;

			/*
			Get closest neighbour
			*/
			OpenMesh::VertexHandle operator()(OpenMesh::HalfedgeHandle toHalfedge) const;
			OpenMesh::VertexHandle operator()(OpenMesh::VertexHandle vertex, OpenMesh::FaceHandle face) const;

		private:
			/*
			KdTree with the gap points
			*/
			GlmPolyMesh gapPoints;

			GlmPolyMeshKdTreeAdaptor gapKdTreeAdaptor{ gapPoints };

			GlmPolyMeshKdTree gapKdTree{ 3, gapKdTreeAdaptor };

			/*
			Reference to the mesh
			*/
			GlmPolyMesh& mesh;

			/*
			Reference to the implicit object
			*/
			Object& object;
		};
	}
}
