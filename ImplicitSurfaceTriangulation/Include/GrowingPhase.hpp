#pragma once

#include "ImplicitObject.hpp"
#include "glmMeshAdaptor.hpp"
#include "glmMeshKdTree.hpp"
#include "Phase.hpp"
#include "Triangle.hpp"

#include <optional>

namespace Implicit
{
	namespace Tessellation
	{
		class GrowingPhase : public Phase<GlmPolyMesh>
		{
		public:
			GrowingPhase(GlmPolyMesh& mesh, Object& object);
			virtual ~GrowingPhase() {}
			virtual void Start() override;
			virtual void Run() override;
			virtual void RunIterations(int iterations) override;
			virtual bool Completed() const override;

			/*
			Set the ratio between triangle side length and local radius of curvature.
			Controls how densely curved areas are triangulated.
			*/
			void SetRho(double rho = 0.2);

			/*
			Longest triangle side *during* the growing phase
			*/
			double GetLongestTriangleSide() const;
		private:
			/*
			Working mesh
			*/
			GlmPolyMesh &mesh;

			/*
			Implicit object
			*/
			Object& object;

			/*
			glm kdTree adaptor
			*/
			GlmPolyMeshKdTreeAdaptor kdTreeAdaptor;

			/*
			kdTree for searching nearby triangles
			*/
			GlmPolyMeshKdTree kdTree;

			/*
			Longest triangle side in the whole mesh
			*/
			double longestSide = 0.0;

			/*
			Handle to the face with the longest side
			*/
			OpenMesh::FaceHandle longestSidedFace;

			/*
			Rho parameter
			*/
			double rho = 0.2;

			/**
			kdTree needs a rebuild before next search
			*/
			bool kdTreeIsDirty = false;

			/*
			Did the mesh change during the last iteration?
			*/
			bool meshChanged = true;

			/*
			Generates the seed triangle.
			*/
			void generateSeedTriangle();

			/**
			Compute radius of curvature at a given point
			*/
			double getRoc(const glm::dvec3& point) const;

			/*
			Add a new face to the mesh
			*/
			OpenMesh::SmartFaceHandle addNewFace(const glm::dvec3& pointA, const glm::dvec3& pointB, const glm::dvec3& pointC);

			/*
			Add a new face to the mesh reusing two existing points
			*/
			OpenMesh::SmartFaceHandle addNewFace(const OpenMesh::VertexHandle& pointA, const OpenMesh::VertexHandle& pointB, const glm::dvec3& pointC, double newTriangleLongestSide);

			/*
			Expand an edge to form an isosceles triangle with the new point. Returns true if succeeded.
			*/
			bool expandEdge(OpenMesh::SmartEdgeHandle edge, OpenMesh::SmartFaceHandle& newFace);

			/*
			Apply ear cutting to the a newly created face
			*/
			bool applyEarCutting(const OpenMesh::SmartFaceHandle& newFace);
		};

	}
}