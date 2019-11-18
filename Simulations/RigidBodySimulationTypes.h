#pragma once
#include "DrawingUtilitiesClass.h"

struct Force
{
	Vec3 location;
	Vec3 force;
};

class Box
{
public:
	Vec3 m_position = Vec3::ZERO;
	Quaternion<double> m_rotation = Quaternion<double>();
	Vec3 m_extents = Vec3(1,1,1);
	Vec3 m_velocity = Vec3::ZERO;
	Vec3 m_angularvelocity = Vec3::ZERO;
	Vec3 m_angularMomentum = Vec3::ZERO;
	Mat4d m_inertiaTensorInverse;
	float m_mass = 1;
	
	Box();
	Box(Vec3 position);
	Box(Vec3 position, Vec3 size);
	Box(Vec3 position, Vec3 size, float mass);
	
	Mat4 asMatrix() const;
	/// Calculate constants
	void initialize(); 
};

