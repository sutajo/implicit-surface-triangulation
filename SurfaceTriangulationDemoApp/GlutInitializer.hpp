#pragma once

#include <GL\glew.h>
#include <GL\freeglut.h>

#include "ImGui/imgui_impl_glut.h"
#include "ImGui/imgui_impl_opengl3.h"

class Glut
{
public:
	struct Settings
	{
		int width = 0;
		int height = 0;
	};

	Glut(int argc, char** argv, const Settings& settings);
	~Glut();
};