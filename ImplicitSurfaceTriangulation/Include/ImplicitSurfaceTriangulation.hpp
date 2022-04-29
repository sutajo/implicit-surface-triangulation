#pragma once

#include "ImplicitObject.hpp"

#include <vector>
#include <glm/glm.hpp>
#include <optional>
#include <functional>

#include "glmMeshAdaptor.hpp"
#include "glmMeshKdTree.hpp"
#include "Triangle.hpp"

namespace Implicit
{
	/**
	 Generates a triangle mesh for a given implicit object
	 */
	class CurvatureTessellator
	{
	public:

		/**
		 Set callbacks used to report state changes during the algorithm 
		 */
		struct Visitor
		{
			/**
			 Create default visitor.
			 */
			Visitor() {}

			virtual ~Visitor() {}

			virtual void SeedTriangleGenerated(const Triangle& triangle) {}
			virtual void NewTriangleGenerated() {}
			virtual void EarCut() {}

			virtual void IterationEnded(bool meshChanged){}
		};

		/**
		 Create new default object
		 */
		CurvatureTessellator(Object& object);

		/**
		 Create new object with Visitor
		 */
		CurvatureTessellator(Object& object, Visitor& visitor);

		/**
		 Set the ratio between triangle side length and local radius of curvature.
		 Controls how densely curved areas are triangulated.
		 */
		void SetRho(double rho);

		/**
		 Runs one iteration of the algorithm. Returns true if the mesh is finished.
		 */
		bool RunIterations(int iterations);

		/**
		 Runs the mesh generation algorithm until completion
		 */
		void ComputeMesh();

		/**
		 Generates the seed triangle.
		 */
		void GenerateSeedTriangle();

		/**
		 Get the current mesh
		 */
		const GlmMesh& GetMesh();

		/**
		 Compute the bounding box using a kdTree
		 */
		const Aabb ComputeBoundingBox();

		/**
		 Get the longest triangle side length
		 */
		double GetLongestTriangleSide() { return longestSide; }

		/**
		 Update vertex normals
		 */
		void UpdateNormals() { mesh.update_normals(); }

	private:
		/**
		 Implicit object
		 */
		Object &object;

		/**
		 Visitor for progress reporting
		 */
		std::optional<std::reference_wrapper<Visitor>> visitor;

		/**
		 Rho parameter 
		 */
		double rho = 0.2;

		/**
		 Generated mesh
		 */
		GlmMesh mesh;

		/**
		 glm kdTree adaptor
		 */
		GlmVec3MeshKdTreeAdaptor kdTreeAdaptor { mesh };

		/**
		 kdTree for searching nearby triangles
		 */
		GlmMeshKdTree kdTree{ 3, kdTreeAdaptor };

		/**
		 kdTree needs a rebuild before next search
		 */
		bool kdTreeIsDirty = false;

		/**
		 Compute radius of curvature at a given point
		 */
		double getRoc(const glm::dvec3& point) const;

		/**
		 Returns a point c such that a,b,c form an equilateral triangle. Gets an equilateral triangle point (hence etp).
		 */
		glm::dvec3 getEtp(const glm::dvec3 &a, const glm::dvec3 &b, const glm::dvec3 &normalizedTangent) const;

		/**
		 Returns a point c such that a,b,c form an isosceles triangle.
		 */
		glm::dvec3 getItp(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& normalizedTangent, double equalSideLength) const;

		/**
		 Add a new face to the mesh
		 */
		void addNewFace(const glm::dvec3& pointA, const glm::dvec3& pointB, const glm::dvec3& pointC);

		/**
		 Add a new face to the mesh reusing two existing points
		 */
		OpenMesh::SmartFaceHandle addNewFace(const OpenMesh::VertexHandle &pointA, const OpenMesh::VertexHandle &pointB, const glm::dvec3& pointC);

		/**
		 Longest triangle side in the whole mesh
		 */
		double longestSide = 0.0;

		/**
		 Handle to the face with the longest side
		 */
		OpenMesh::FaceHandle longestSidedFace;

		/**
		 Expand an edge to form an isosceles triangle with the new point. Returns true if succeeded.
		 */
		bool expandEdge(OpenMesh::SmartEdgeHandle edge, OpenMesh::SmartFaceHandle &newFace);


		/*
		* Apply ear cutting to the a newly created face
		*/
		bool applyEarCutting(const OpenMesh::SmartFaceHandle& newFace);
	};
};