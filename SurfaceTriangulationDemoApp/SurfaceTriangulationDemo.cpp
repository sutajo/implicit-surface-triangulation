#include "SurfaceTriangulationDemo.hpp"
#include "Common/DebugOpenGL.hpp"

#include "fieldFunctions.hpp"
#include "ImplicitSphere.hpp"
#include "ImplicitTorus.hpp"
#include "ImplicitBlend.hpp"
#include "ImplicitTransform.hpp"
#include "ImplicitTranslate.hpp"
#include "ImplicitScale.hpp"

#include <imgui.h>
#include <glm/gtx/vector_angle.hpp>
#include <format>

using namespace std::chrono;

SurfaceTriangulationDemo* instance = nullptr;

SurfaceTriangulationDemo::SurfaceTriangulationDemo(int argc, char* argv[]) : glut(argc, argv, Glut::Settings{1280, 1280})
{
	DebugOpenGL::init();

	glClearColor(30.0f / 255.0f, 30.0f / 255.0f, 30.0f / 255.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LINE_SMOOTH);
	SetLineWidth();

	meshShader.loadShader(GL_VERTEX_SHADER, "shaders/mesh.vs");
	meshShader.loadShader(GL_FRAGMENT_SHADER, "shaders/mesh.fs");

	if( !meshShader.compile() )
		throw std::runtime_error("Could not compile shader");

	// static Implicit::Sphere s(metaballFunction);

	implicitObjects.push_back({ "Sphere", std::make_unique<Implicit::Sphere>(linearFunction) });
	implicitObjects.push_back({ "Torus" , std::make_unique<Implicit::Torus>()                }) ;
	// implicitObjects.push_back({ "Translate" , std::make_unique<Implicit::Scale>(&s, glm::dvec3(0.5, 0.5, 0.5)) });
	
	/*
	GlmMesh omesh;
	omesh.request_vertex_normals();
	OpenMesh::IO::Options opt;
	if (!OpenMesh::IO::read_mesh(omesh, "SampleModels/nanosuit.obj", opt))
		throw std::runtime_error("Could not load sample model");

	if (!opt.check(OpenMesh::IO::Options::VertexNormal))
	{
		omesh.request_face_normals();
		omesh.update_normals();
		omesh.release_face_normals();
	}
	mesh.Update(omesh);
	*/

	SetTessellatedObect(GetSelectedObject());
	UpdateMesh();

	instance = this;

	glutDisplayFunc(onDisplay);
	glutKeyboardFunc(onKeyboardDown);
	glutKeyboardUpFunc(onKeyboardUp);
	glutMouseFunc(onMouse);
	glutMotionFunc(onMotion);
	glutIdleFunc(onIdle);
	glutReshapeFunc(onReshape);
}

int SurfaceTriangulationDemo::Run()
{
	glutMainLoop();
	return 0;
}

void SurfaceTriangulationDemo::onMotion(int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	ImGui_ImplGLUT_MotionFunc(x, y);

	if (!io.WantCaptureMouse)
	{
		const int dx = x - instance->input.lastMouseX, dy = y - instance->input.lastMouseY;
		if (instance->input.rightButtonHeld)
		{
			// Kamera távolság állítás
			instance->camera.SetView(instance->camera.GetEye() - dy * io.DeltaTime * instance->camera.GetFw(), instance->camera.GetAt(), instance->camera.GetUp());
		}
		else
		{
			if(instance->algorithmSettings.freeCamera)
				instance->camera.MouseMove(dx, dy);
			else {
				instance->mesh.RotateInCameraSpace(dx, dy, instance->camera);
				instance->closestNeighbours.RotateInCameraSpace(dx, dy, instance->camera);
				instance->bb.RotateInCameraSpace(dx, dy, instance->camera);
			}
		}
		
		instance->SetLineWidth();

		instance->input.lastMouseX = x;
		instance->input.lastMouseY = y;
	}
	glutPostRedisplay();
}

void SurfaceTriangulationDemo::onMouse(int button, int state, int x, int y)
{
	ImGui_ImplGLUT_MouseFunc(button, state, x, y);

	if (button == GLUT_RIGHT_BUTTON)
		instance->input.rightButtonHeld = state == GLUT_DOWN;

	instance->input.lastMouseX = x;
	instance->input.lastMouseY = y;
}

void SurfaceTriangulationDemo::onKeyboardDown(unsigned char key, int pX, int pY)
{
	ImGuiIO& io = ImGui::GetIO();
	ImGui_ImplGLUT_KeyboardFunc(key, pX, pY);
	if (!io.WantCaptureKeyboard && instance->algorithmSettings.freeCamera)
	{
		instance->camera.KeyboardDown(key);
	}
	glutPostRedisplay();
}

void SurfaceTriangulationDemo::onKeyboardUp(unsigned char key, int pX, int pY)
{
	ImGuiIO& io = ImGui::GetIO();
	ImGui_ImplGLUT_KeyboardUpFunc(key, pX, pY);
	if (!io.WantCaptureKeyboard && instance->algorithmSettings.freeCamera)
	{
		instance->camera.KeyboardUp(key);
	}
	glutPostRedisplay();
}

void SurfaceTriangulationDemo::onIdle()
{
	glutPostRedisplay();
}

void SurfaceTriangulationDemo::onReshape(int width, int height)
{
	ImGui_ImplGLUT_ReshapeFunc(width, height);
	instance->camera.SetProj(glm::radians(60.0f), width / (float)height, 0.01f, 1000.0f);
}

void SurfaceTriangulationDemo::SeedTriangleGenerated(const Triangle& seed)
{
	log.AddLog("Seed triangle generated\n%s\n", seed.to_string().c_str());
	UpdateMesh();
}

void SurfaceTriangulationDemo::NewTriangleGenerated()
{
	log.AddLog("New triangle generated");
}

void SurfaceTriangulationDemo::EarCut()
{
	log.AddLog("Ear cut");
}

void SurfaceTriangulationDemo::IterationEnded(bool meshChanged)
{
	if (meshChanged)
	{
		if (algorithmSettings.realTimeUpdate &&
			(pendingIterations % ITERATIONS_PER_FRAME) == 0)
		{
			UpdateMesh();
		}
	}
	else
	{
		pendingIterations = 0;
		UpdateMesh();
	}
}

void SurfaceTriangulationDemo::ShowClickedFace(bool* p_open)
{
	const float GRID_STEP = 64.0f;
	const int gridSize = 6;
	
	if (!ImGui::Begin("Face view", p_open, ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::End();
		return;
	}
	
	static ImVector<ImVec2> points;
	static ImVec2 scrolling(0.0f, 0.0f);
	static bool adding_line = false;


	ImGui::Text("Clicked face id: %d", clickedFace.idx());

	auto vertices = clickedFace.vertices().to_array<3>();
	auto glmMesh = tessellator->GetMesh();
	auto triangle = Triangle{ glmMesh.point(vertices[0]), glmMesh.point(vertices[1]), glmMesh.point(vertices[2]) };

	ImGui::Text("A (Idx %d): (%3f, %3f, %3f)\nB (Idx %d): (%3f, %3f, %3f)\nC (Idx %d): (%3f, %3f, %3f)",
		vertices[0].idx(), triangle.a.x, triangle.a.y, triangle.a.z,
		vertices[1].idx(), triangle.b.x, triangle.b.y, triangle.b.z,
		vertices[2].idx(), triangle.c.x, triangle.c.y, triangle.c.z
	);

	ImGui::Text("Mouse Left: drag to measure distance,\nMouse Right: drag to scroll, click for context menu.");

	// Using InvisibleButton() as a convenience 1) it will advance the layout cursor and 2) allows us to use IsItemHovered()/IsItemActive()
	ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
	ImVec2 canvas_sz = ImVec2(1.7 * GRID_STEP*gridSize, GRID_STEP * gridSize);   // Resize canvas to what's available
	if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
	if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
	ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

	// Draw border and background color
	ImGuiIO& io = ImGui::GetIO();
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
	draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

	// This will catch our interactions
	ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
	const bool is_hovered = ImGui::IsItemHovered(); // Hovered
	const bool is_active = ImGui::IsItemActive();   // Held
	const ImVec2 origin(canvas_p0.x + scrolling.x, canvas_p0.y + scrolling.y); // Lock scrolled origin
	const ImVec2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);

	// Add first and second point
	if (is_hovered && !adding_line && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
	{
		points.push_back(mouse_pos_in_canvas);
		points.push_back(mouse_pos_in_canvas);
		adding_line = true;
	}
	if (adding_line)
	{
		points.back() = mouse_pos_in_canvas;
		if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
		{
			adding_line = false;
			points.clear();
		}
	}

	// Pan (we use a zero mouse threshold when there's no context menu)
	// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
	if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right))
	{
		scrolling.x += io.MouseDelta.x;
		scrolling.y += io.MouseDelta.y;
	}

	// Context menu (under default mouse threshold)
	ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
	if (drag_delta.x == 0.0f && drag_delta.y == 0.0f)
		ImGui::OpenPopupOnItemClick("context", ImGuiPopupFlags_MouseButtonRight);
	if (ImGui::BeginPopup("context"))
	{
		if (adding_line)
			points.resize(points.size() - 2);
		adding_line = false;
		if (ImGui::MenuItem("Remove one", NULL, false, points.Size > 0)) { points.resize(points.size() - 2); }
		if (ImGui::MenuItem("Remove all", NULL, false, points.Size > 0)) { points.clear(); }
		ImGui::EndPopup();
	}

	// Draw grid + all lines in the canvas
	draw_list->PushClipRect(canvas_p0, canvas_p1, true);

	// Grid
	for (float x = fmodf(scrolling.x, GRID_STEP); x < canvas_sz.x; x += GRID_STEP)
		draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
	for (float y = fmodf(scrolling.y, GRID_STEP); y < canvas_sz.y; y += GRID_STEP)
		draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(200, 200, 200, 40));

	double zoom = 1.0;
	const double longestSide = triangle.GetLongestSide();
	const double canvasSmallestSide = std::min(canvas_sz.x, canvas_sz.y);

	zoom = GRID_STEP * 4.0 / longestSide;
	auto sides = triangle.GetSideLengths();

	const glm::dvec2 axis1 = glm::dvec2(1.0, 0.0);
	const double angle = glm::angle(glm::normalize(triangle.b - triangle.a), glm::normalize(triangle.c - triangle.a));
	const glm::dvec2 axis2 = glm::normalize(glm::rotate(axis1, angle));

	const glm::dvec2 mid = (zoom * std::get<0>(sides) * axis1 + zoom * std::get<2>(sides) * axis2) / 2.;
	
	// Triangle
	const auto faceColor = mesh.GetColorFromFaceIndex(clickedFace.idx());
	auto canvas_center = ImVec2(origin.x + canvas_sz.x / 2. - mid.x / 2., origin.y + canvas_sz.y / 2. - mid.y / 2.);

	const ImVec2 gridTriangleA = ImVec2(canvas_center.x, canvas_center.y);
	const ImVec2 gridTriangleB = ImVec2(canvas_center.x + zoom * std::get<0>(sides) * axis1.x, canvas_center.y + zoom * std::get<0>(sides) * axis1.y);
	const ImVec2 gridTriangleC = ImVec2(canvas_center.x + zoom * std::get<2>(sides) * axis2.x, canvas_center.y + zoom * std::get<2>(sides) * axis2.y);
	draw_list->AddTriangleFilled(
		gridTriangleA,
		gridTriangleB,
		gridTriangleC,
		ImColor(faceColor.r, faceColor.g, faceColor.b)
	);

	auto find_edge = [&](OpenMesh::VertexHandle v1, OpenMesh::VertexHandle v2)
	{
		OpenMesh::HalfedgeHandle heh = glmMesh.find_halfedge(v1, v2);
		if (heh.is_valid()) {
			return glmMesh.edge_handle(heh);
		}
		else {
			return OpenMesh::EdgeHandle();
		}
	};

	auto find_boundary_halfedge = [&](OpenMesh::VertexHandle v1, OpenMesh::VertexHandle v2)
	{
		OpenMesh::HalfedgeHandle heh = glmMesh.find_halfedge(v1, v2);
		if (heh.is_valid()) {
			if (glmMesh.is_boundary(heh))
				return heh;
			else
				return glmMesh.opposite_halfedge_handle(heh);
		}
		else {
			return OpenMesh::HalfedgeHandle();
		}
	};

	auto boundaryColor = ImColor(243, 255, 18);
	auto innerColor = ImColor(77, 250, 2); 

	// Triangle edges
	draw_list->AddLine(gridTriangleA, gridTriangleB, glmMesh.is_boundary(find_edge(vertices[0], vertices[1])) ? boundaryColor : innerColor, 3.0f);
	draw_list->AddLine(gridTriangleB, gridTriangleC, glmMesh.is_boundary(find_edge(vertices[1], vertices[2])) ? boundaryColor : innerColor, 3.0f);
	draw_list->AddLine(gridTriangleA, gridTriangleC, glmMesh.is_boundary(find_edge(vertices[0], vertices[2])) ? boundaryColor : innerColor, 3.0f);

	// Triangle side lengths
	std::string distStr = std::format("Idx: {}, Len: {:.4f}", find_boundary_halfedge(vertices[0], vertices[1]).idx(), std::get<0>(sides));
	auto size = ImGui::GetFont()->CalcTextSizeA(ImGui::GetFontSize(), FLT_MAX, FLT_MAX, distStr.c_str());
	draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f, ImVec2((gridTriangleA.x + gridTriangleB.x) / 2.f - size.x / 2.f, (gridTriangleA.y + gridTriangleB.y) / 2.f - size.y * 2.f), IM_COL32(235, 168, 52, 255), distStr.c_str());
	distStr = std::format("Idx: {}, Len: {:.4f}", find_boundary_halfedge(vertices[1], vertices[2]).idx(), std::get<1>(sides));
	draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f, ImVec2((gridTriangleB.x + gridTriangleC.x) / 2.f + 15.f, (gridTriangleB.y + gridTriangleC.y) / 2.f), IM_COL32(235, 168, 52, 255), distStr.c_str());
	size = ImGui::GetFont()->CalcTextSizeA(ImGui::GetFontSize(), FLT_MAX, FLT_MAX, distStr.c_str());
	distStr = std::format("Idx: {}, Len: {:.4f}", find_boundary_halfedge(vertices[0], vertices[2]).idx(), std::get<2>(sides));
	draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f, ImVec2((gridTriangleA.x + gridTriangleC.x) / 2.f - size.x*1.6f, (gridTriangleA.y + gridTriangleC.y) / 2.f), IM_COL32(235, 168, 52, 255), distStr.c_str()); 

	distStr = std::format("{}", vertices[0].idx());
	size = ImGui::GetFont()->CalcTextSizeA(ImGui::GetFontSize(), FLT_MAX, FLT_MAX, distStr.c_str());
	draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f, ImVec2(gridTriangleA.x - size.x * 2.0f, gridTriangleA.y - 15.0f), IM_COL32(59, 222, 255, 255), distStr.c_str());
	draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f, ImVec2(gridTriangleB.x + 10.0f , gridTriangleB.y - 15.0f), IM_COL32(59, 222, 255, 255), std::format("{}", vertices[1].idx()).c_str());
	draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f, ImVec2(gridTriangleC.x - 5.0f  , gridTriangleC.y + 10.0f), IM_COL32(59, 222, 255, 255), std::format("{}", vertices[2].idx()).c_str());

	// Lines
	for (int n = 0; n < points.Size; n += 2)
	{
		draw_list->AddLine(ImVec2(origin.x + points[n].x, origin.y + points[n].y), ImVec2(origin.x + points[n + 1].x, origin.y + points[n + 1].y), IM_COL32(235, 64, 52, 200), 2.0f); 
		const float distanceX = points[n].x - points[n + 1].x;
		const float distanceY = points[n].y - points[n + 1].y;
		const float distance = sqrt(distanceX * distanceX + distanceY * distanceY) / zoom;
		draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize()*1.8f, ImVec2(origin.x + (points[n].x + points[n + 1].x) / 2.f, origin.y + (points[n].y + points[n + 1].y) / 2.f), IM_COL32(235, 64, 52, 255), std::to_string(distance).c_str()); 
	}

	draw_list->PopClipRect();
	
	ImGui::End();
}

void SurfaceTriangulationDemo::DrawUI()
{
	using namespace ImGui;

	ImGuiIO& io = GetIO();
	BeginGroup();

	SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
	SetNextWindowSize(ImVec2(0, 0), ImGuiCond_FirstUseEver);
	Begin("Info");
		if (CollapsingHeader("Performance", ImGuiTreeNodeFlags_DefaultOpen))
		{
			Text("Framerate: %f", io.Framerate);
			Text("Frametime: %f ms", io.DeltaTime * 1000.0f);
		}

		if (CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen))
		{
			Checkbox("Free camera", &algorithmSettings.freeCamera);
			SameLine();
			if (Button("Reset"))
			{
				Camera defaultCam;
				camera.SetView(defaultCam.GetEye(), defaultCam.GetAt(), defaultCam.GetUp());
			}
			auto eye = camera.GetEye();
			Text("Position: (%f, %f, %f)", eye.x, eye.y, eye.z);
			auto fw = glm::normalize(camera.GetAt() - eye);
			Text("Forward: (%f, %f, %f)", fw.x, fw.y, fw.z);
		}

		if (CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGuiIO& io = GetIO();
			{
				auto aabb = GetSelectedObject().GetBoundingBox();
				auto min = aabb.min(), max = aabb.max();
				Text("Bounding box: (%.3f,%.3f,%.3f) - (%.3f, %.3f, %.3f)", min.x, min.y, min.z, max.x, max.y, max.z);
			}
			Text("Number of triangles: %zu", mesh.GetTriangleCount());
			Text("Number of vertices: %zu", mesh.GetVertexCount());
			Text("Longest triangle side: %f", tessellator->GetLongestTriangleSide());
			if (Button("Reset orientation"))
			{
				mesh.ResetRotation();
				bb.ResetRotation();
			}
			SameLine();
			Checkbox("Show bounding box of the surface", &algorithmSettings.showBoundingBox);
			if (Combo("Face visualization mode", (int*)&algorithmSettings.faceVisualization, "Id\0Normals\0Face creation method\0"))
			{
				if(algorithmSettings.faceVisualization == FaceVisualization::Normal)
					tessellator->UpdateNormals();

				UpdateMesh();
			}

			if (IsMouseClicked(ImGuiMouseButton_Middle) && algorithmSettings.faceVisualization == FaceVisualization::Id)
			{
				clickedFace = QueryClickedFace();
			}
			if (clickedFace.is_valid())
			{
				Spacing();
				ShowClickedFace(nullptr);
			}
		}

		if (CollapsingHeader("Tessellation", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (CollapsingHeader("Input", ImGuiTreeNodeFlags_DefaultOpen))
			{
				PushItemWidth(150);
				if (SliderFloat("Triangle side length / local radius of curvature (Rho)", &algorithmSettings.rho, 0.0f, 0.4f) && tessellator)
				{
					tessellator->SetRho(algorithmSettings.rho);
				}
				PopItemWidth();

				static std::string comboOptions;
				if (comboOptions.empty())
				{
					for (auto& implicitObj : implicitObjects)
					{
						comboOptions += implicitObj.first;
						comboOptions += '\0';
					}
				}

				if (Combo("Select implicit surface", &algorithmSettings.selectedObjectIndex, comboOptions.c_str()))
				{
					SetTessellatedObect(GetSelectedObject());
				}
			}
			Separator();
			if (CollapsingHeader("Control", ImGuiTreeNodeFlags_DefaultOpen))
			{
				try
				{
					if (Button("Reset mesh"))
					{
						SetTessellatedObect(GetSelectedObject());
					}
					SameLine();
					if (Button("Generate seed"))
					{
						tessellator->GenerateSeedTriangle();
						tessellator->UpdateNormals();
					}
					SameLine();
					PushItemWidth(150);
					SliderInt("N", &algorithmSettings.nIterations, 1, 10000);
					SameLine();
					Checkbox("Update mesh in real time", &algorithmSettings.realTimeUpdate);
					
					Separator();
					if (pendingIterations > 0)
					{
						TextColored(ImVec4(0.f, 1.f, 0.f, 1.f), "Pending iterations: %d", pendingIterations);
						SameLine();
						if (Button("Cancel") && tessellator)
						{
							pendingIterations = 0;
						}
						Separator();
					}

					if (Button("Run iteration") && tessellator)
					{
						tessellator->RunIterations(1);
					}
					SameLine();
					if (Button("Run N iterations"))
					{
						pendingIterations += algorithmSettings.nIterations;
					}
					SameLine();
					if (Button("Run tessellation until completion"))
					{
						tessellator->ComputeMesh();
					}
				}
				catch (const std::exception& e)
				{
					log.AddLog("An exception occured:\n%s\n\n", e.what());
				}
			}
		}
	End();

	ImGui::SetNextWindowPos(ImVec2(0.0f, io.DisplaySize.y), 0, ImVec2(0.0f, 1.0f));
	ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);
	log.Draw("Tessellation log");

	EndGroup();
}

void SurfaceTriangulationDemo::DrawMesh(Mesh& mesh, bool drawLines, bool drawFill)
{
	auto& world = mesh.GetWorldMat();
	instance->meshShader.enable();
	instance->meshShader.setUniformMat4("MVP", instance->camera.GetViewProj() * world);
	instance->meshShader.setUniformMat4("world", world);
	instance->meshShader.setUniformMat4("worldIT", glm::inverse(glm::transpose(world)));
	if (drawFill || drawLines)
	{
		instance->meshShader.setUniform1i("uUseDiffuseShading", false);
		instance->meshShader.setUniform3f("uColor", 0.8f, 0.8f, 0.8f);
		instance->meshShader.setUniform1i("uUniformColor", false);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		drawLines ? mesh.RenderAsLines() : mesh.Render();
	}
	else
		instance->meshShader.setUniform1i("uUseDiffuseShading", true);

	if (!drawLines)
	{
		instance->meshShader.setUniform3f("uColor", 0.3f, 0.8f, 0.8f);
		//instance->meshShader.setUniform3f("uColor", 245 / 255.f, 182 / 255.f, 66 / 255.f);
		instance->meshShader.setUniform1i("uUniformColor", true);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		drawLines ? mesh.RenderAsLines() : mesh.Render();
	}

	meshShader.disable();
}

void SurfaceTriangulationDemo::SetLineWidth()
{
	glLineWidth(std::max(7.0f / glm::distance(camera.GetEye(), camera.GetAt()), 1.0f));
}

void SurfaceTriangulationDemo::UpdateMesh()
{
	mesh.Update(mesh.GetMeshVertices(tessellator->GetMesh(), algorithmSettings.faceVisualization), GetSelectedObject().GetCenterVertex());
	if (tessellator->ClosestNeighboursComputed())
	{
		closestNeighbours.Update(mesh.GetLineVertices(tessellator->GetMesh()), GetSelectedObject().GetCenterVertex());
	}
}

void SurfaceTriangulationDemo::SetTessellatedObect(Implicit::Object &object)
{
	tessellator.emplace(object, *this);
	tessellator->SetRho(algorithmSettings.rho);
	auto aabb = object.GetBoundingBox();
	auto min = aabb.min() + object.GetCenterVertex(), max = aabb.max() + object.GetCenterVertex();
	auto w = max.x - min.x, h = max.y - min.y, d = max.z - min.z;
	bb.SetCenter(object.GetCenterVertex());
	bb.Resize(w / 2.0f, h / 2.0f, d / 2.0f);
	UpdateMesh();
	log.Clear();
	clickedFace = OpenMesh::SmartFaceHandle();
}

Implicit::Object& SurfaceTriangulationDemo::GetSelectedObject()
{
	return *implicitObjects[algorithmSettings.selectedObjectIndex].second;
}

void SurfaceTriangulationDemo::RunPendingIterations()
{
	if (pendingIterations > 0)
	{
		const int iterations = std::min(ITERATIONS_PER_FRAME, pendingIterations);
		const bool changed = !tessellator->RunIterations(iterations);
		pendingIterations -= iterations;
		if (changed &&
			algorithmSettings.realTimeUpdate)
		{
			tessellator->UpdateNormals();
			UpdateMesh();
		}
	}

}

OpenMesh::SmartFaceHandle SurfaceTriangulationDemo::QueryClickedFace()
{
	glFlush();
	glFinish();

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	ImGuiIO& io = ImGui::GetIO();
	float RGBA[3];
	const GLint y = glutGet(GLUT_WINDOW_HEIGHT) - static_cast<int>(io.MousePos.y);
	glReadPixels(static_cast<int>(io.MousePos.x), y, 1, 1, GL_RGB, GL_FLOAT, &RGBA);

	return OpenMesh::SmartFaceHandle(
		mesh.GetFaceIndexFromColor((int)(RGBA[0] * 255.f), (int)(RGBA[1] * 255.f), (int)(RGBA[2] * 255.f)),
		&tessellator->GetMesh()
	);
}

void SurfaceTriangulationDemo::onDisplay()
{
	// ImGUI frame begin
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGLUT_NewFrame();

	// Update camera
	instance->camera.Update(ImGui::GetIO().DeltaTime);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Run pending iterations
	instance->RunPendingIterations();

	// Drawing
	instance->DrawMesh(instance->mesh, false);
	instance->DrawUI();

	if (instance->algorithmSettings.showBoundingBox)
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_FRONT);
		instance->DrawMesh(instance->bb, false, false);
		glDisable(GL_CULL_FACE);
	}
	if (instance->tessellator->ClosestNeighboursComputed())
	{
		instance->DrawMesh(instance->closestNeighbours, true, false);
	}

	// ImGUI frame end
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glutSwapBuffers();
}

