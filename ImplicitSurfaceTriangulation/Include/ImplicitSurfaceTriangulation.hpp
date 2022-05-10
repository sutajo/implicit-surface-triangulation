#pragma once

#include "ImplicitObject.hpp"

#include <vector>
#include <glm/glm.hpp>
#include <optional>
#include <functional>
#include <deque>
#include <set>

#include "glmMeshAdaptor.hpp"
#include "glmMeshKdTree.hpp"
#include "Triangle.hpp"
#include "Plane.hpp"

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
		 Runs n iterations of the growing phase. Returns true if the phase is completed.
		 */
		bool RunGrowingIterations(int iterations);

		/**
		 Runs n iterations of the filling phase. Returns true if the phase is completed.
		 */
		bool RunFillingIterations(int iterations);

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
		const GlmPolyMesh& GetMesh();

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

		/**
		 Returns whether closest neighbour relationships have been computed for boundary vertices
		 */
		bool ClosestNeighboursComputed() { return closestNeighboursComputed; }

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
		 Max triangle side length
		 */
		std::optional<double> maxSideLength;

		/**
		 Generated mesh
		 */
		GlmPolyMesh mesh;

		/**
		 glm kdTree adaptor
		 */
		GlmPolyMeshKdTreeAdaptor kdTreeAdaptor { mesh };

		/**
		 kdTree for searching nearby triangles
		 */
		GlmPolyMeshKdTree kdTree{ 3, kdTreeAdaptor };

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
		 Returns a plane corresponding to the halfedge.
		*/
		Plane getPlaneFromHalfEdge(const OpenMesh::SmartHalfedgeHandle& heh) const;

		/**
		 Add a new face to the mesh
		 */
		OpenMesh::SmartFaceHandle addNewFace(const glm::dvec3& pointA, const glm::dvec3& pointB, const glm::dvec3& pointC);

		/**
		 Add a new face to the mesh reusing two existing points
		 */
		OpenMesh::SmartFaceHandle addNewFace(const OpenMesh::VertexHandle &pointA, const OpenMesh::VertexHandle &pointB, const glm::dvec3& pointC, double newTriangleLongestSide);

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

		/*
		* Add a mesh vertex to the gap
		*/
		OpenMesh::VertexHandle addGapVertex(OpenMesh::VertexHandle vertex, bool rebuildKdTree = false);

		/*
		* Remove a mesh vertex from the gap
		*/
		void removeGapVertex(OpenMesh::VertexHandle vertex);

		/*
		* Compute the closest neighbour of the to vertex of the halfedge
		*/
		OpenMesh::VertexHandle computeClosestNeighbour(OpenMesh::SmartHalfedgeHandle heh);

		/*
		* Compute closest neighbour relationships for the filling phase
		*/
		void computeClosestNeighbours();

		/*
		* Stores whether closest neigbours have been computed
		*/
		bool closestNeighboursComputed = false;

		/*
		* Is the to vertex of the halfedge convex?
		*/
		bool isConvex(OpenMesh::SmartHalfedgeHandle toHalfedge);

		/*
		* Is the vertex a neigbour of the "to" vertex of the halfedge?
		*/
		bool isNeighbour(OpenMesh::SmartHalfedgeHandle toHalfedge, OpenMesh::VertexHandle vertex);

		/*
		* Is the vertex part of a bridge?
		*/
		bool isBridge(OpenMesh::VertexHandle vertex);

		/*
		* Gap list for filling phase
		*/
		std::deque<OpenMesh::FaceHandle> gaps;

		/*
		* Filling phase heuristic 1: Small Polygon Filling
		*/
		bool SmallPolygonFilling(OpenMesh::FaceHandle gap);

		/*
		* Filling phase heuristic 2: Subdivision on Bridges
		*/
		bool SubdivisionOnBridges(OpenMesh::FaceHandle gap);

		/*
		* Filling phase heuristic 3: X Filling
		*/
		bool XFilling(OpenMesh::FaceHandle gap);

		/*
		* Filling phase heuristic 4: Ear Filling
		*/
		bool EarFilling(OpenMesh::FaceHandle gap);

		/*
		* Filling phase heuristic 5: Convex Polygon Filling
		*/
		bool ConvexPolygonFilling(OpenMesh::FaceHandle gap);

		/*
		* Filling phase heuristic 6: Relaxed Ear Filling
		*/
		bool RelaxedEarFilling(OpenMesh::FaceHandle gap);

		/*
		* Filling phase heuristic 7: Concave Vertex Bisection
		*/
		bool ConcaveVertexBisection(OpenMesh::FaceHandle gap);

		/*
		* KdTree with the gap points
		*/
		GlmPolyMesh gap;
		GlmPolyMeshKdTreeAdaptor gapKdTreeAdaptor{ gap };
		GlmPolyMeshKdTree gapKdTree{ 3, gapKdTreeAdaptor };
	};
};