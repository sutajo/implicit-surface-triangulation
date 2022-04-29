/*
 * ImplicitObject
 *
 * File: 	ImplicitObject.cpp
 * Author:	Evan Wilde		<etcwilde@uvic.ca>
 * Date:	Feb 18 2015
 */

#include "ImplicitObject.hpp"

using namespace Implicit;

Object::Object():
	m_iso(0.5)
{ }

Object::Object(double iso) :
	m_iso(iso)
{ }


void Object::SetIso(double iso)
{
	m_iso = iso;
}

double Object::GetIso()
{
	return m_iso;
}


glm::dvec3 Object::Project(const glm::dvec3& p)
{
	return project(p);
}

glm::dvec3 Object::Project(const glm::dvec3& p, glm::dvec3 d)
{
	return project(p, d);
}

bool Object::Intersect(const glm::dvec3& origin, glm::dvec3& cp)
{
	return GetBoundingBox().intersect(origin, Normal(origin), cp);
}

bool Object::Intersect(const glm::dvec3& origin, glm::dvec3 direction,
		glm::dvec3& cp)
{
	return GetBoundingBox().intersect(origin, direction, cp);
	// First intersect the bounding box
	// If it does, then project the rest of the way.
	/*bool i = GetBoundingBox().intersect(origin, direction, cp);
	if (i)
	{
		std::cout << "cp: " << cp.x << " " << cp.y << " " << cp.z << '\n';
		std::cout << "Evaluates to: " << Evaluate(cp) << '\n';
		std::cout << "FF: " << FieldValue(cp) << '\n';
	}
	if (!i) return false;

	cp = project(cp, direction);
	std::cout << "Final cp: " << cp.x << " " << cp.y << " " << cp.z << '\n';
	std::cout << "Evaluates to: " << Evaluate(cp) << '\n';
	std::cout << "FF: " << FieldValue(cp) << '\n';

	return f_equ(Evaluate(cp), 0); */
}

void Object::Curvature(const glm::dvec3& pt, double& k1, double& k2)
{
	glm::dvec3 grad = Normal(pt);
	glm::dmat3 H = hessian(pt);
	glm::dmat3 C = surfaceCurvature(H);
	double nfsqr = glm::dot(grad, grad);

	double gaussian = glm::dot(grad, (grad * C)) / (nfsqr*nfsqr + FLT_EPSILON);
	double mean = (glm::dot(grad, (grad * H)) - nfsqr *
			(H[0][0] + H[1][1] + H[2][2]))
			/ (2.f * nfsqr + FLT_EPSILON);
	double det = std::sqrt(abs(mean * mean - gaussian));
	k1 = mean + det;
	k2 = mean - det;
	/*
#ifdef DEBUG
	std::cout << "Gaussian: " << gaussian << '\t' << "Mean: " << mean
		<< '\n' << "Determinant: " << det << '\n'
		<< "K1: " << k1 << '\t' << "K2: " << k2 << '\n';
#endif
	*/
}


Aabb Object::GetBoundingBox()
{
	return m_bounds;
}


void Object::getDeltas(double& dx, double& dy, double& dz, const glm::dvec3& pt,
		float eps)
{
	double x = std::abs(pt.x);
	double y = std::abs(pt.y);
	double z = std::abs(pt.z);

	dx = eps * std::max(x, 1.);
	dy = eps * std::max(y, 1.);
	dz = eps * std::max(z, 1.);
	volatile double tmpx = x + dx;
	volatile double tmpy = y + dy;
	volatile double tmpz = z + dz;
	dx = tmpx - x;
	dy = tmpy - y;
	dz = tmpz - z;
}

glm::dmat3 Object::hessian(const glm::dvec3& pt)
{
	static const double EPSILON = FLT_EPSILON;
	double hx, hy, hz;
	getDeltas(hx, hy, hz, pt, EPSILON);

	glm::dvec3 dx(hx, 0, 0);
	glm::dvec3 dy(0, hy, 0);
	glm::dvec3 dz(0, 0, hz);

	glm::dvec3 gxp = Normal(pt + dx);
	glm::dvec3 gxm = Normal(pt - dx);
	glm::dvec3 gyp = Normal(pt + dy);
	glm::dvec3 gym = Normal(pt - dy);
	glm::dvec3 gzp = Normal(pt + dz);
	glm::dvec3 gzm = Normal(pt - dz);


	double inv_2hx = 0.5f/hx;
	double inv_2hy = 0.5f/hy;
	double inv_2hz = 0.5f/hz;

	double hxx = inv_2hx * (gxp.x - gxm.x);
	double hyy = inv_2hy * (gyp.y - gym.y);
	double hzz = inv_2hz * (gzp.z - gzm.z);
	double hxy = 0.5f*(inv_2hx*(gxp.y - gxm.y) + inv_2hy*(gyp.x - gyp.x));
	double hyz = 0.5f*(inv_2hy*(gyp.z - gym.z) + inv_2hz*(gzp.y - gzp.y));
	double hxz = 0.5f*(inv_2hz*(gzp.x - gzm.x) + inv_2hx*(gxp.z - gxp.z));

	glm::dvec3 col0(hxx, hxy, hxz);
	glm::dvec3 col1(hxy, hyy, hxy);
	glm::dvec3 col2(hxz, hyz, hzz);
	glm::dmat3 H(col0, col1, col2);
	return H;
}

glm::dmat3 Object::surfaceCurvature(const glm::dmat3& m)
{
	double Mxx = m[0][0];
	double Myy = m[1][1];
	double Mzz = m[2][2];
	double Mxy = m[0][1];
	double Mxz = m[0][2];
	double Myz = m[1][2];
	glm::dmat3 C;
	C[0][0] = Myy * Mzz - Myz * Myz;
	C[1][1] = Mxx * Mzz - Mxz * Mxz;
	C[2][2] = Mxx * Myy - Mxy * Mxy;
	C[1][0] = Mxy * Mxz - Mxy * Mzz;
	C[2][0] = Mxy * Myz - Myy * Mxz;
	C[2][1] = Mxy * Mxz - Mxx * Myz;
	C[0][1] = m[1][0];
	C[0][2] = m[2][0];
	C[1][2] = m[2][1];
	return C;
}

double Object::findRoot(const glm::dvec3& point, glm::dvec3 direction, double id)
{
	direction = glm::normalize(direction);
	double ret_val;
	double xi;
	double xi1 = 0;
	double xi2 = id;

	double fxi1 = Evaluate(point + (direction * xi1));
	double fxi2 = Evaluate(point + (direction * xi2));

#ifdef DEBUG
	unsigned int iteration = 0;
#endif
	for (unsigned int i = 0; i < FIND_ROOT_ITERS; ++i)
	{

		xi = xi1 - fxi1 * ((xi1 - xi2)/(fxi1 - fxi2));
		if (fxi1 == fxi2)
		{
			ret_val = xi1;
#ifdef DEBUG
			iteration = i;
#endif
			break;
		}
		xi2 = xi1; xi1 = xi;
		fxi2 = fxi1;
		fxi1 = Evaluate(point + (direction * xi1));
		if (f_equ(fxi1, 0))
		{
#ifdef DEBUG
			iteration = i;
#endif
			ret_val = xi1;
			break;
		}
	}
	/*
#ifdef DEBUG
	std::cout << iteration << " iterations ";
#endif
	*/
	return ret_val;
}

glm::dvec3 Implicit::Object::findRootBetween(glm::dvec3 innerPoint, glm::dvec3 outerPoint, int maxiterations)
{
	static const double EPSILON = 0.00001;

	double f1 = Evaluate(innerPoint);
	if (abs(f1) < EPSILON)
		return innerPoint;

	double f2 = Evaluate(outerPoint);
	if (abs(f2) < EPSILON)
		return outerPoint;

	if(f1*f2 > 0.0f)
		throw std::runtime_error("Points are not appropriate");

	glm::vec3 midpoint = (innerPoint + outerPoint) / 2.;
	double fm = Evaluate(midpoint);
	int i;
	for (i = 0; i < maxiterations && abs(fm) > EPSILON; ++i)
	{
		if (f1 * fm < 0.0)
		{
			outerPoint = midpoint;
			f2 = fm;
		}
		else
		{
			innerPoint = midpoint;
			f1 = fm;
		}
		midpoint = (innerPoint + outerPoint) / 2.;
		fm = Evaluate(midpoint);
	}

	if (i == maxiterations)
		throw std::runtime_error("No roots found");
	else
		return midpoint;
}

void Object::getTangentSpace(const glm::dvec3& N, glm::dvec3& T, glm::dvec3& B)
	const
{
	if (N.x > 0.5f || N.y > 0.5f) T = glm::dvec3(N.y, -N.x, 0.f);
	else T = glm::dvec3(-N.z, 0.f, N.x);
	B = glm::cross(N, T);
	T = glm::normalize(T);
	B = glm::normalize(B);
}

/*
* glm::vec3 Object::project(const glm::vec3& pt)
{
	// the point + some distance along the gradient
	// Gives us the point on the surface
	const float f = Evaluate(pt);
	auto sign = signbit(f) ? -1.0f : 1.0f;
	auto n = Normal(pt);
	n *= sign;
	float d = std::abs(f);
	int iters = 0;
	while (Evaluate(pt + d * n) * f >= 0.0f && iters < 100)
	{
		d *= 2.f;
		++iters;
	}
	if (iters == 100)
		throw std::runtime_error("Failed to find points with opposite field signs");

	return findRootBetween(pt, pt + d*n);
}
*/

glm::dvec3 Object::project(const glm::dvec3& pt)
{
	// the point + some distance along the gradient
	// Gives us the point on the surface
	auto sign = signbit(Evaluate(pt)) ? -1.0f : 1.0f;
	auto n = Normal(pt);
	n *= sign;
	return pt + (n * findRoot(pt, n));
}

glm::dvec3 Object::project(const glm::dvec3& p, glm::dvec3 d)
{
	return p + d * findRoot(p, d);
}

double Object::DistanceFromSurface(const glm::dvec3& pt)
{
	const glm::dvec3 direction = glm::normalize(pt - GetCenterVertex());
	const glm::dvec3 surface_point = direction * findRoot(GetCenterVertex(), direction);
	return glm::length(pt - surface_point);
}

