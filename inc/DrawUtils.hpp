/*
 * DrawUtils
 *
 * File: 	DrawUtils.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 22 2015
 */

#ifndef DRAW_UTILS_HPP
#define DRAW_UTILS_HPP

#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#ifdef DEBUG
#include <iostream>
#endif

/**
 * \brief Directly draws a quadralateral to opengl
 * \param v1 vertex 1
 * \param v2 vertex 2
 * \param v3 vertex 3
 * \param v4 vertex 4
 * \param n1 normal for vertex 1
 * \param n2 normal for vertex 2
 * \param n3 normal for vertex 3
 * \param n4 normal for vertex 4
 */
void drawQuad(const glm::vec3& v1,
		const glm::vec3& v2,
		const glm::vec3& v3,
		const glm::vec3& v4,
		const glm::vec3& n1,
		const glm::vec3& n2,
		const glm::vec3& n3,
		const glm::vec3& n4);

void drawBox(const glm::vec3& max, const glm::vec3& min);
#endif//DRAW_UTILS_HPP
