#pragma once

#include "ImplicitObject.hpp"
#include "glmMeshAdaptor.hpp"
#include "glmMeshKdTree.hpp"
#include "Phase.hpp"
#include <deque>

namespace Implicit
{
	namespace Tessellation
	{
		class FillingPhase : public Phase<GlmPolyMesh>
		{
		private:
			GlmPolyMesh& mesh;
			Object& object;
		public:
			FillingPhase(GlmPolyMesh& mesh, Object& object);
			virtual ~FillingPhase() {}
			virtual void Start() override;
			virtual void RunIterations(int iterations) override;
			virtual bool Completed() const override;
		private:

			/*
			Add a mesh vertex to the gap
			*/
			OpenMesh::VertexHandle addGapVertex(OpenMesh::VertexHandle vertex, bool rebuildKdTree = false);

			/*
			Remove a mesh vertex from the gap
			*/
			void removeGapVertex(OpenMesh::VertexHandle vertex);

			/*
			Compute the closest neighbour of the to vertex of the halfedge
			*/
			OpenMesh::VertexHandle computeClosestNeighbour(OpenMesh::SmartHalfedgeHandle heh);

			/*
			Compute closest neighbour relationships for the filling phase
			*/
			void computeClosestNeighbours();

			/*
			Is the to vertex of the halfedge convex?
			*/
			bool isConvex(OpenMesh::SmartHalfedgeHandle toHalfedge);

			/*
			Is the vertex a neigbour of the "to" vertex of the halfedge?
			*/
			bool isNeighbour(OpenMesh::SmartHalfedgeHandle toHalfedge, OpenMesh::VertexHandle vertex);

			/*
			Is the vertex part of a bridge?
			*/
			bool isBridge(OpenMesh::VertexHandle vertex);

			/*
			Gap list for filling phase
			*/
			std::deque<OpenMesh::FaceHandle> gaps;

			/*
			Filling phase heuristic 1: Small Polygon Filling
			*/
			bool SmallPolygonFilling(OpenMesh::FaceHandle gap);

			/*
			Filling phase heuristic 2: Subdivision on Bridges
			*/
			bool SubdivisionOnBridges(OpenMesh::FaceHandle gap);

			/*
			Filling phase heuristic 3: X Filling
			*/
			bool XFilling(OpenMesh::FaceHandle gap);

			/*
			Filling phase heuristic 4: Ear Filling
			*/
			bool EarFilling(OpenMesh::FaceHandle gap);

			/*
			Filling phase heuristic 5: Convex Polygon Filling
			*/
			bool ConvexPolygonFilling(OpenMesh::FaceHandle gap);

			/*
			Filling phase heuristic 6: Relaxed Ear Filling
			*/
			bool RelaxedEarFilling(OpenMesh::FaceHandle gap);

			/*
			Filling phase heuristic 7: Concave Vertex Bisection
			*/
			bool ConcaveVertexBisection(OpenMesh::FaceHandle gap);

			/*
			KdTree with the gap points
			*/
			GlmPolyMesh gapPoints;

			GlmPolyMeshKdTreeAdaptor gapKdTreeAdaptor{ gapPoints };

			GlmPolyMeshKdTree gapKdTree{ 3, gapKdTreeAdaptor };
		};
	}
}
