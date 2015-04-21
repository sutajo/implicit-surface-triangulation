/*
 * ImplicitSystem
 *
 * File: 	ImplicitSystem.hpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Mar 12 2015
 */

#ifndef IMPLICIT_SYSTEM_HPP
#define IMPLICIT_SYSTEM_HPP

/**
 * \brief Implicit objects
 *
 * All non-explicitly defined shapes.
 *
 * Implicit objects are geometry defined by math rather than explicit vertex,
 * edge, and face data.
 */
namespace Implicit {};

#include "Aabb.hpp"

#include "fieldFunctions.hpp"

#include "ImplicitObject.hpp"

// Primitive Objects
#include "ImplicitPrimitive.hpp"
#include "ImplicitSphere.hpp"
#include "ImplicitLine.hpp"
#include "ImplicitTorus.hpp"

// Binary Operations
#include "ImplicitOperator.hpp"
#include "ImplicitUnion.hpp"
#include "ImplicitBlend.hpp"
#include "ImplicitIntersect.hpp"

// Unary Operations
#include "ImplicitTransform.hpp"
#include "ImplicitTranslate.hpp"
#include "ImplicitScale.hpp"
#include "ImplicitRotate.hpp"

#endif //IMPLICIT_SYSTEM_HPP
