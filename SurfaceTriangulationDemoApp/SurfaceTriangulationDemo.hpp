﻿#pragma once

#include "Common/Camera.hpp"
#include "Common/shader.hpp"
#include "GlutInitializer.hpp"
#include "Mesh.hpp"
#include "Box.hpp"
#include "ImplicitSurfaceTriangulation.hpp"
#include "ImGuiLog.hpp"

#include <chrono>
#include <memory>


struct AlgorithmVisualizationSettings
{
	// Kiterjesztésre váró élek: kék
	// Kiterjesztett élek: zöld és piros között átmenet attól függően, hogy mennyire jól sikerült az él
	bool showBoundingBox = true;
	bool freeCamera = false;

	float rho = 0.1f;
	int selectedObjectIndex = 0;
	int nIterations = 15;
	bool realTimeUpdate = true;
	FaceVisualization faceVisualization = FaceVisualization::Id;
};

struct InputState
{
	int lastMouseX = 0, lastMouseY = 0;
	bool rightButtonHeld = false;
};

class SurfaceTriangulationDemo : public Implicit::CurvatureTessellator::Visitor
{
private:
	// Glut handle
	Glut glut;

	// Triangle mesh and shader
	Mesh mesh;
	Shader meshShader;

	// Bounding box
	Box bb{ 1.0f, 1.0f, 1.0f };

	// Camera
	Camera camera;

	// Input state
	InputState input;

	// Selectable implicit objects
	std::vector<std::pair<std::string, std::unique_ptr<Implicit::Object>>> implicitObjects;

	// Algorithm settings
	AlgorithmVisualizationSettings algorithmSettings;

	// The tessellation algorithm
	std::optional<Implicit::CurvatureTessellator> tessellator;

	// Glut callbacks
	static void onMouse(int button, int state, int x, int y);
	static void onMotion(int x, int y);
	static void onKeyboardDown(unsigned char key, int pX, int pY);
	static void onKeyboardUp(unsigned char key, int pX, int pY);
	static void onDisplay();
	static void onIdle();
	static void onReshape(int width, int height);

	// Visitor functions
	ImGuiLog log;
	virtual void SeedTriangleGenerated(const Triangle& seed) override;
	virtual void NewTriangleGenerated() override;
	virtual void EarCut() override;
	virtual void IterationEnded(bool meshChanged) override;

	void DrawUI();
	void DrawMesh(Mesh& mesh, bool drawFill = true);
	void SetLineWidth();

	void UpdateMesh();
	void SetTessellatedObect(Implicit::Object &object);
	Implicit::Object& GetSelectedObject();

	const int ITERATIONS_PER_FRAME = 15;
	int pendingIterations = 0;
	void RunPendingIterations();

	OpenMesh::SmartFaceHandle QueryClickedFace();
	OpenMesh::SmartFaceHandle clickedFace;
	void ShowClickedFace(bool *p_open);

public:
	SurfaceTriangulationDemo(int argc, char* argv[]);
	int Run();

	virtual ~SurfaceTriangulationDemo() {}
};