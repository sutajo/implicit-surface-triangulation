/*
 * Aabb
 *
 * File: Aabb.cpp
 * Auth: Evan Wilde					<etcwilde@uvic.ca>
 * Date: Mar 22 2015
 */

#include "Aabb.hpp"



void Aabb::reset()
{
	const double max = INFINITY;
	bounds[1] = glm::dvec3(-max, -max, -max);
	bounds[0] = glm::dvec3(max, max, max);
	memset(m_ray_cache, 0, AABB_RAY_CACHE_SIZE);
}

void Aabb::include(const glm::dvec3& point)
{
	if (point.x > bounds[1].x) bounds[1].x = point.x;
	if (point.y > bounds[1].y) bounds[1].y = point.y;
	if (point.z > bounds[1].z) bounds[1].z = point.z;

	if (point.x < bounds[0].x) bounds[0].x = point.x;
	if (point.y < bounds[0].y) bounds[0].y = point.y;
	if (point.z < bounds[0].z) bounds[0].z = point.z;
}

void Aabb::compute(const std::vector<glm::dvec3>& verts)
{
	if (verts.size() < 1) return;
	bounds[1] = verts[0];
	bounds[0] = verts[0];

	for (unsigned int i = 1; i < verts.size(); ++i)
	{
		include(verts[i]);
	}
}

void Aabb::compute(std::list<glm::dvec3>& verts)
{
	if (verts.size() < 1) return;
	bounds[1] = *(verts.begin());
	bounds[0] = *(verts.begin());

	for (std::list<glm::dvec3>::iterator i = ++verts.begin();
			i != verts.end(); i++) include(*i);
}

void Aabb::add(const Aabb& other)
{
	const glm::dvec3& max_other = other.bounds[1];
	const glm::dvec3& min_other = other.bounds[0];

	if (max_other.x > bounds[1].x) bounds[1].x = max_other.x;
	if (max_other.y > bounds[1].y) bounds[1].y = max_other.y;
	if (max_other.z > bounds[1].z) bounds[1].z = max_other.z;

	if (min_other.x < bounds[0].x) bounds[0].x = min_other.x;
	if (min_other.y < bounds[0].y) bounds[0].y = min_other.y;
	if (min_other.z < bounds[0].z) bounds[0].z = min_other.z;
}

void Aabb::expand(double factor)
{
	glm::dvec3 diff = (bounds[1] - bounds[0]) * (0.5 * factor);
	glm::dvec3 center = (bounds[1] + bounds[0]) * 0.5;
	bounds[1] = center + diff;
	bounds[0] = center + diff;
}

void Aabb::transform(const glm::dmat4& t)
{
	// Translated points
	std::vector<glm::dvec3> trans(8);
	trans[0]=glm::dvec3(t*glm::dvec4(bounds[1].x, bounds[1].y, bounds[1].z, 1.));
	trans[1]=glm::dvec3(t*glm::dvec4(bounds[1].x, bounds[0].y, bounds[1].z, 1.));
	trans[2]=glm::dvec3(t*glm::dvec4(bounds[0].x, bounds[1].y, bounds[1].z, 1.));
	trans[3]=glm::dvec3(t*glm::dvec4(bounds[0].x, bounds[0].y, bounds[1].z, 1.));
	trans[4]=glm::dvec3(t*glm::dvec4(bounds[1].x, bounds[1].y, bounds[0].z, 1.));
	trans[5]=glm::dvec3(t*glm::dvec4(bounds[1].x, bounds[0].y, bounds[0].z, 1.));
	trans[6]=glm::dvec3(t*glm::dvec4(bounds[0].x, bounds[1].y, bounds[0].z, 1.));
	trans[7]=glm::dvec3(t*glm::dvec4(bounds[0].x, bounds[0].y, bounds[0].z, 1.));
	compute(trans);
}

double Aabb::volume() const
{
	glm::dvec3 width = bounds[1] - bounds[0];
	return width.x* width.y* width.z;
}

double Aabb::surfaceArea() const
{
	glm::dvec3 width = bounds[1] - bounds[0];
	return 2.f*(width.x * width.y + width.z * width.y + width.z * width.x);
}

double Aabb::size() const
{
	glm::dvec3 width = bounds[1] - bounds[0];
	(width.x < width.y) && (width.x = width.y);
	(width.x < width.z) && (width.x  = width.z);
	return width.x;
}

bool Aabb::overlap(const Aabb& o) const
{
	if (bounds[0].x > o.bounds[1].x) return false;
	if (bounds[0].y > o.bounds[1].y) return false;
	if (bounds[0].z > o.bounds[1].z) return false;
	if (bounds[1].x > o.bounds[0].x) return false;
	if (bounds[1].y > o.bounds[0].y) return false;
	if (bounds[1].z > o.bounds[0].z) return false;
	return true;
}

bool Aabb::contains(const glm::dvec3& p) const
{
	return
		((bounds[0].x < p.x) && (p.x < bounds[1].x)) &
		((bounds[0].y < p.y) && (p.y < bounds[1].y)) &
		((bounds[0].z < p.z) && (p.z < bounds[1].z));
}

const glm::dvec3& Aabb::min() const
{
	return bounds[0];
}
const glm::dvec3& Aabb::max() const
{
	return bounds[1];
}

#define XZ 0b00000001
#define XY 0b00000010
#define YZ 0b00000100
Aabb::Axis Aabb::majorAxis() const
{
	unsigned char result = 0;
	const glm::vec3 m = bounds[1] - bounds[0];
	(m.x > m.z) && (result |= XZ);
	(m.y > m.z) && (result |= YZ);
	(m.x > m.y) && (result |= XY);
	return (result & XZ) ?
		((result & XY) ? Axis::X : Axis::Y) :
		((result & YZ) ? Axis::Y : Axis::Z);
}

Aabb::Axis Aabb::minorAxis() const
{
	unsigned char result = 0;
	const glm::vec3 m = bounds[1] - bounds[0];
	(m.x < m.z) && (result |= XZ);
	(m.y < m.z) && (result |= YZ);
	(m.x < m.y) && (result |= XY);
	return (result & XZ) ?
		((result & XY) ? Axis::X : Axis::Y) :
		((result & YZ) ? Axis::Y : Axis::Z);
}

#undef XZ
#undef YX
#undef YZ


unsigned int Aabb::hash_ray(const Aabb::AABB_RAY& R)
{
	return hash_ray(R.origin, R.direction);
}

unsigned int Aabb::hash_ray(const glm::dvec3& origin, const glm::dvec3& direction)
{
	std::hash<float> hash_float;
	long unsigned int x;
	long unsigned int mult = RAY_HASH_MULTIPLIER;
	long int y;
	x = 0x345678UL;

	y = hash_float(origin.x);
	if (y == -1) return -1;
	x = (x ^ y) * mult;
	mult += (long int)(82520UL + 12);

	y = hash_float(origin.y);
	if (y == -1) return -1;
	x = (x ^ y) * mult;
	mult += (long int)(82520UL + 12);

	y = hash_float(origin.z);
	if (y == -1) return -1;
	x = (x ^ y) * mult;
	mult += (long int)(82520UL + 12);

	y = hash_float(direction.x);
	if (y == -1) return -1;
	x = (x ^ y) * mult;
	mult += (long int)(82520UL + 12);

	y = hash_float(direction.y);
	if (y == -1) return -1;
	x = (x ^ y) * mult;
	mult += (long int)(82520UL + 12);

	y = hash_float(direction.z);
	if (y == -1) return -1;
	x = (x ^ y) * mult;
	mult += (long int)(82520UL + 12);

	x += 97531UL;
	if (x == (unsigned long int) -1)
		x = -2;
	return x;
}



/**
 * http://www.cs.utah.edu/~awilliam/box/box.pdf
 */
bool Aabb::intersect(const glm::dvec3& origin, const glm::dvec3& direction,
		glm::dvec3& hit_point)
{
	// Check that we don't already have a ray in the cache
	// Cached ray
	const AABB_RAY cr = m_ray_cache[hash_ray(origin, direction) % AABB_RAY_CACHE_SIZE];
	AABB_RAY r;
	// Check that this isn't simply a hash collision
	if(cr.origin.x == origin.x &&
	cr.origin.y == origin.y &&
	cr.origin.z == origin.z &&
	cr.direction.x == direction.x &&
	cr.direction.y == direction.y &&
	cr.direction.z == direction.z)
		r = cr;
	else
	{
		r = AABB_RAY(origin, direction);
		m_ray_cache[hash_ray(r) % AABB_RAY_CACHE_SIZE] = r;
	}

	// Using test ray
	double tmin, tmax, tymin, tymax, tzmin, tzmax;
	tmin = (bounds[r.sign[0]].x - r.origin.x) * r.inv_direction.x;
	tmax = (bounds[1-r.sign[0]].x - r.origin.x) * r.inv_direction.x;
	tymin = (bounds[r.sign[1]].y - r.origin.y) * r.inv_direction.y;
	tymax = (bounds[1-r.sign[1]].y - r.origin.y) * r.inv_direction.y;
	if ((tmin > tymax) || (tymin > tmax)) return false;
	if (tymin > tmin) tmin = tymin;
	if (tymax < tmax) tmax = tymax;
	tzmin = (bounds[r.sign[2]].z - r.origin.z) * r.inv_direction.z;
	tzmax = (bounds[1 - r.sign[2]].z - r.origin.z) * r.inv_direction.z;
	if ((tmin > tzmax) || (tzmin > tmax)) return false;
	if (tzmin > tmin) tmin = tzmin;
	if (tzmax < tmax) tmax = tzmax;
	hit_point = (direction * tmin) + origin;
	return tmax > tmin;
}
