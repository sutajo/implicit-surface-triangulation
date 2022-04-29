#include <iostream>
#include "Camera.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <math.h>

/// <summary>
/// Initializes a new instance of the <see cref="gCamera"/> class.
/// </summary>
Camera::Camera(void) : m_eye(0.0f, 0.0f, 3.0f), m_at(0.0f), m_up(0.0f, 1.0f, 0.0f), m_speed(16.0f), m_goFw(0), m_goRight(0), m_slow(false)
{
	SetView( m_eye, m_at, m_up );

	m_dist = glm::length( m_at - m_eye );	

	SetProj(glm::radians(60.0f), 640/480.0f, 0.01f, 1000.0f);
}

Camera::Camera(glm::vec3 _eye, glm::vec3 _at, glm::vec3 _up) : m_speed(16.0f), m_goFw(0), m_goRight(0), m_dist(10), m_slow(false)
{
	SetView(_eye, _at, _up);
}

Camera::~Camera(void)
{
}

void Camera::SetView(glm::vec3 _eye, glm::vec3 _at, glm::vec3 _up)
{
	m_eye	= _eye;
	m_at	= _at;
	m_up	= _up;

	m_fw  = glm::normalize( m_at - m_eye  );
	m_st = glm::normalize( glm::cross( m_fw, m_up ) );

	m_dist = glm::length( m_at - m_eye );	

	m_u = atan2f( m_fw.z, m_fw.x );
	m_v = acosf( m_fw.y );
}

void Camera::SetProj(float _angle, float _aspect, float _zn, float _zf)
{
	m_matProj = glm::perspective( _angle, _aspect, _zn, _zf);
	m_matViewProj = m_matProj * m_viewMatrix;
}

const glm::mat4 &Camera::GetViewMatrix() const
{
	return m_viewMatrix;
}

void Camera::Update(float _deltaTime)
{
	m_eye += (m_goFw*m_fw + m_goRight*m_st)*m_speed*_deltaTime;
	m_at  += (m_goFw*m_fw + m_goRight*m_st)*m_speed*_deltaTime;

	m_viewMatrix = glm::lookAt( m_eye, m_at, m_up);
	m_matViewProj = m_matProj * m_viewMatrix;
}

void Camera::UpdateUV(float du, float dv)
{
	m_u		+= du;
	m_v		 = glm::clamp<float>(m_v + dv, 0.1f, 3.1f);

	m_at = m_eye + m_dist*glm::vec3(	cosf(m_u)*sinf(m_v), 
										cosf(m_v), 
										sinf(m_u)*sinf(m_v) );

	m_fw = glm::normalize( m_at - m_eye );
	m_st = glm::normalize( glm::cross( m_fw, m_up ) );
}

void Camera::SetSpeed(float _val)
{
	m_speed = _val;
}

void Camera::Resize(int _w, int _h)
{
	SetProj(glm::radians(60.0f), _w/(float)_h, 0.01f, 1000.0f);
}

void Camera::KeyboardDown(unsigned char key)
{
	if (glutGetModifiers() & GLUT_ACTIVE_SHIFT)
	{
		if ( !m_slow )
		{
			m_slow = true;
			m_speed /= 4.0f;
		}
	}

	switch ( ::tolower(key) )
	{
	case 'w':
			m_goFw = 1;
		break;
	case 's':
			m_goFw = -1;
		break;
	case 'a':
			m_goRight = -1;
		break;
	case 'd':
			m_goRight = 1;
		break;
	}
}

void Camera::KeyboardUp(unsigned char key)
{
	float current_speed = m_speed;
	if ((glutGetModifiers() & GLUT_ACTIVE_SHIFT) == 0)
	{
		if (m_slow)
		{
			m_slow = false;
			m_speed *= 4.0f;
		}
	}
	switch ( ::tolower(key) )
	{
	case 'w':
	case 's':
			m_goFw = 0;
		break;
	case 'a':
	case 'd':
			m_goRight = 0;
		break;
	}
}

void Camera::MouseMove(int relx, int rely)
{
	UpdateUV(relx/100.0f, rely/100.0f);
}

void Camera::LookAt(glm::vec3 _at)
{
	SetView(m_eye, _at, m_up);
}

