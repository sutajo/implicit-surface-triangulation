#pragma once

#include <glm/glm.hpp>

#include <GL/glew.h>
#include <GL/freeglut.h>

class Camera
{
public:
	Camera(void);
	Camera(glm::vec3 _eye, glm::vec3 _at, glm::vec3 _up);
	~Camera(void);

	/// <summary>
	/// Gets the view matrix.
	/// </summary>
	/// <returns>The 4x4 view matrix</returns>
	const glm::mat4 &GetViewMatrix() const;

	void Update(float _deltaTime);

	void SetView(glm::vec3 _eye, glm::vec3 _at, glm::vec3 _up);
	void SetProj(float _angle, float _aspect, float _zn, float _zf); 
	void LookAt(glm::vec3 _at);

	void SetSpeed(float _val);
	const glm::vec3 &GetEye() const
	{
		return m_eye;
	}

	const glm::vec3 &GetAt() const
	{
		return m_at;
	}

	const glm::vec3 &GetUp() const
	{
		return m_up;
	}

	const glm::mat4 &GetProj() const
	{
		return m_matProj;
	}

	const glm::vec3 &GetFw() const
	{
		return m_fw;
	}

	const glm::mat4 &GetViewProj() const
	{
		return m_matViewProj;
	}

	void Resize(int _w, int _h);

	void KeyboardDown(unsigned char key);
	void KeyboardUp(unsigned char key);
	void MouseMove(int relx, int rely);

private:
	/// <summary>
	/// Updates the UV.
	/// </summary>
	/// <param name="du">The du, i.e. the change of spherical coordinate u.</param>
	/// <param name="dv">The dv, i.e. the change of spherical coordinate v.</param>
	void UpdateUV(float du, float dv);

	/// <summary>
	///  The traversal speed of the camera
	/// </summary>
	float		m_speed;
	/// <summary>
	/// The view matrix of the camera
	/// </summary>
	glm::mat4	m_viewMatrix;

	glm::mat4	m_matViewProj;

	bool	m_slow;

	/// <summary>
	/// The camera position.
	/// </summary>
	glm::vec3	m_eye;

	/// <summary>
	/// The vector pointing upwards
	/// </summary>
	glm::vec3	m_up;

	/// <summary>
	/// The camera look at point.
	/// </summary>
	glm::vec3	m_at;

	/// <summary>
	/// The u spherical coordinate of the spherical coordinate pair (u,v) denoting the
	/// current viewing direction from the view position m_eye. 
	/// </summary>
	float	m_u;

	/// <summary>
	/// The v spherical coordinate of the spherical coordinate pair (u,v) denoting the
	/// current viewing direction from the view position m_eye. 
	/// </summary>
	float	m_v;

	/// <summary>
	/// The distance of the look at point from the camera. 
	/// </summary>
	float	m_dist;

	/// <summary>
	/// The unit vector pointing towards the viewing direction.
	/// </summary>
	glm::vec3	m_fw;
	/// <summary>
	/// The unit vector pointing to the 'right'
	/// </summary>
	glm::vec3	m_st;

	glm::mat4	m_matProj;

	float	m_goFw;
	float	m_goRight;
};

