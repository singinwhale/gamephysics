#include "RigidBodySystem.h"

void RigidBodySystem::initialize()
{
	for (auto& body : m_rigid_bodies)
	{
		body.initialize();
	}
}

void RigidBodySystem::tick(float deltaSeconds)
{
	// I. integrate properties of rigid bodies
	for (auto &body : m_rigid_bodies)
	{
		// Following code implements slide 14 of gp-lecture05-rigid-bodies-3D
		// TODO: sum all forces
		float totalForce = 0.0;
		// TODO: sum contribution of forces w.r.t mass points
		Vec3 torque = Vec3(0.0);

		// Integrate linear parts
		body.m_position += deltaSeconds * body.m_velocity;
		body.m_velocity += deltaSeconds * totalForce/body.m_mass;
		
		// Angular velocity
		const auto& w = body.m_angularvelocity;

		// Update angular rotation
		Quaternion<double>& derivative = Quaternion<double>(0.0, w.x, w.y, w.z);
		body.m_rotation += (deltaSeconds / 2.0)*derivative*body.m_rotation;

		// Calculate inertia for current rotation
		Mat4d inertiaInverse = body.m_rotation.getRotMat().inverse()*body.inertiaTensorInverse*body.m_rotation.getRotMat();
	
		// Update angular momentum ...
		body.m_angularMomentum += deltaSeconds * torque;
		// .. and use it to extract angular velocity from angular momentum and store it
		body.m_angularvelocity = inertiaInverse * body.m_angularMomentum;

	}
	
	// II. Detect collisions & apply impluses if neccessary

	
}
