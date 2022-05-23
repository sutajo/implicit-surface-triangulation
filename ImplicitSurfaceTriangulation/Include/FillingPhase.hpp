#pragma once

#include "ClosestNeighbours.hpp"
#include "ImplicitObject.hpp"
#include "glmMeshAdaptor.hpp"
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
			ClosestNeighbours closestNeighbours;
		public:
			FillingPhase(GlmPolyMesh& mesh, Object& object);
			virtual ~FillingPhase() {}
			virtual void Start() override;
			virtual void RunIterations(int iterations) override;
			virtual bool Completed() const override;
			const ClosestNeighbours& GetClosestNeighbours() const;
		private:
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
			bool EarFilling(OpenMesh::FaceHandle gap, bool relaxed);

			/*
			Filling phase heuristic 5: Convex Polygon Filling
			*/
			bool ConvexPolygonFilling(OpenMesh::FaceHandle gap);

			/*
			Filling phase heuristic 6: Relaxed Ear Filling
			See EarFilling
			*/

			/*
			Filling phase heuristic 7: Concave Vertex Bisection
			*/
			bool ConcaveVertexBisection(OpenMesh::FaceHandle gap);
		};
	}
}
