#include "Aabb.hpp"

std::ostream& operator << (std::ostream& os, const glm::vec3& v)
{
	os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
	return os;
}

void Aabb::reset()
{
	const float max = INFINITY;
	m_maxima = glm::vec3(-max, -max, -max);
	m_minima = glm::vec3(max, max, max);
}

void Aabb::include(const glm::vec3& point)
{
	if (point.x > m_maxima.x) m_maxima.x = point.x;
	if (point.y > m_maxima.y) m_maxima.y = point.y;
	if (point.z > m_maxima.z) m_maxima.z = point.z;

	if (point.x < m_minima.x) m_minima.x = point.x;
	if (point.y < m_minima.y) m_minima.y = point.y;
	if (point.z < m_minima.z) m_minima.z = point.z;
}

void Aabb::compute(const std::vector<glm::vec3>& verts)
{
	if (verts.size() < 1) return;
	m_maxima = verts[0];
	m_minima = verts[0];

	for (unsigned int i = 1; i < verts.size(); ++i)
	{
		include(verts[i]);
	}
}

void Aabb::compute(std::list<glm::vec3>& verts)
{
	if (verts.size() < 1) return;
	m_maxima = *(verts.begin());
	m_minima = *(verts.begin());

	for (std::list<glm::vec3>::iterator i = ++verts.begin();
			i != verts.end(); i++) include(*i);
}




void Aabb::add(const Aabb& other)
{
	const glm::vec3& max_other = other.m_maxima;
	const glm::vec3& min_other = other.m_minima;

	if (max_other.x > m_maxima.x) m_maxima.x = max_other.x;
	if (max_other.y > m_maxima.y) m_maxima.y = max_other.y;
	if (max_other.z > m_maxima.z) m_maxima.z = max_other.z;

	if (min_other.x < m_minima.x) m_minima.x = min_other.x;
	if (min_other.y < m_minima.y) m_minima.y = min_other.y;
	if (min_other.z < m_minima.z) m_minima.z = min_other.z;
}

void Aabb::expand(float factor)
{
	glm::vec3 diff = (m_maxima - m_minima) * (0.5f * factor);
	glm::vec3 center = (m_maxima + m_minima) * 0.5f;
	m_maxima = center + diff;
	m_minima = center + diff;
}

void Aabb::transform(const glm::mat4& t)
{
	// Translated points
	std::vector<glm::vec3> trans(8);
	trans[0]=glm::vec3(t*glm::vec4(m_maxima.x, m_maxima.y, m_maxima.z, 1.f));
	trans[1]=glm::vec3(t*glm::vec4(m_maxima.x, m_minima.y, m_maxima.z, 1.f));
	trans[2]=glm::vec3(t*glm::vec4(m_minima.x, m_maxima.y, m_maxima.z, 1.f));
	trans[3]=glm::vec3(t*glm::vec4(m_minima.x, m_minima.y, m_maxima.z, 1.f));
	trans[4]=glm::vec3(t*glm::vec4(m_maxima.x, m_maxima.y, m_minima.z, 1.f));
	trans[5]=glm::vec3(t*glm::vec4(m_maxima.x, m_minima.y, m_minima.z, 1.f));
	trans[6]=glm::vec3(t*glm::vec4(m_minima.x, m_maxima.y, m_minima.z, 1.f));
	trans[7]=glm::vec3(t*glm::vec4(m_minima.x, m_minima.y, m_minima.z, 1.f));
	compute(trans);
}

float Aabb::volume() const
{
	glm::vec3 width = m_maxima - m_minima;
	return width.x* width.y* width.z;
}

float Aabb::surfaceArea() const
{
	glm::vec3 width = m_maxima - m_minima;
	return 2.f*(width.x * width.y + width.z * width.y + width.z * width.x);
}

float Aabb::size() const
{
	glm::vec3 width = m_maxima - m_minima;
	(width.x < width.y) && (width.x = width.y);
	(width.x < width.z) && (width.x  = width.z);
	return width.x;
}

bool Aabb::overlap(const Aabb& o) const
{
	if (m_minima.x > o.m_maxima.x) return false;
	if (m_minima.y > o.m_maxima.y) return false;
	if (m_minima.z > o.m_maxima.z) return false;
	if (m_maxima.x > o.m_minima.x) return false;
	if (m_maxima.y > o.m_minima.y) return false;
	if (m_maxima.z > o.m_minima.z) return false;
	return true;
}

bool Aabb::intersect(const glm::vec3& origin, const glm::vec3& direction, float& distance)
{
	const glm::vec3 d_inv(1/direction.x, 1/direction.y, 1/direction.z);
	const float tx1 = (m_minima.x - origin.x) * d_inv.x;
	const float tx2 = (m_maxima.x - origin.x) * d_inv.x;
	float tmin;
	float tmax;

	tmin = tx1 <= tx2 ? tx1 : tx2;
	tmax = tx1 > tx2 ? tx1 : tx2;

	const float ty1 = (m_minima.y - origin.y) * d_inv.y;
	const float ty2 = (m_maxima.y - origin.y) * d_inv.y;

	tmin = std::max(tmin, std::min(ty1, ty2));
	tmax = std::min(tmax, std::max(ty1, ty2));

	const float tz1 = (m_minima.z - origin.z) * d_inv.z;
	const float tz2 = (m_maxima.z - origin.z) * d_inv.z;

	tmin = std::max(tmin, std::min(tz1, tz2));
	tmax = std::min(tmax, std::max(tz1, tz2));

#ifdef DEBUG
	std::cout << "tmax: " << tmax << " tmin: " << tmin << '\n';
#endif

	return tmin <= tmax;

}
