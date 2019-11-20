#include "RigidBodySystem.h"
#include "collisionDetect.h"

void RigidBodySystem::initialize()
{
	for (auto& body : m_rigid_bodies)
	{
		body.initialize();
	}
}

double helperCalculateImpulse(Vec3 relativeVelocity, Vec3 normal, Vec3 positionA, Vec3 positionB, float massA, float massB, Mat4 inertiaA, Mat4 inertiaB, float bounciness = 1.0)
{
	const auto nominator = -(1 + bounciness)*(dot(normal, relativeVelocity));
	const auto denominator = 1 / massA + 1 / massB + dot((cross((inertiaA*(cross(positionA, normal))), normal) + cross((inertiaA*(cross(positionA, normal))), normal)), normal);
	return nominator / denominator;
}

void RigidBodySystem::tick(float deltaSeconds)
{
	// I. integrate properties of rigid bodies
	for (auto &body : m_rigid_bodies)
	{
		// Following code implements slide 14 of gp-lecture05-rigid-bodies-3D
		// TODO: sum all forces
		Vec3 totalForce = m_constantForce;
		// TODO: sum contribution of forces w.r.t mass points
		Vec3 torque = Vec3(0.0);
		
		for (auto& application : body.m_forceApplications)
		{
			// position x force
			torque += cross(application.first, application.second);
		}
		// Clear impulses
		body.m_forceApplications.clear();

		totalForce += torque;
		std::cout << "Force: " << totalForce << " - " << torque << std::endl;

		// Integrate linear parts
		body.m_position += deltaSeconds * body.m_velocity;
		body.m_velocity += deltaSeconds * (totalForce/body.m_mass);


		std::cout << "[Mass: " << body.m_mass << "] Position: " << body.m_position << " - s [" << body.m_velocity << "]" << std::endl;
		
		// Angular velocity
		const auto& w = body.m_angularvelocity;

		// Update angular rotation
		Quaternion<double>& derivative = Quaternion<double>(w.x, w.y, w.z,0.0);
		body.m_rotation += (deltaSeconds / 2.0)*derivative*body.m_rotation;
		body.m_rotation = body.m_rotation.unit();

		// Calculate inertia for current rotation
		body.m_inertiaTensorInverse = body.m_rotation.getRotMat().inverse()*body.m_inertiaTensorInverseZero*body.m_rotation.getRotMat();
	
		// Update angular momentum ...
		body.m_angularMomentum += deltaSeconds * torque;
		// .. and use it to extract angular velocity from angular momentum and store it
		body.m_angularvelocity = body.m_inertiaTensorInverse * body.m_angularMomentum;
	}
	
	// II. Detect collisions & apply impluses if neccessary
	for (size_t i = 0; i < m_rigid_bodies.size(); i++)
	{
		auto& referenceBody = m_rigid_bodies[i];
		GamePhysics::Mat4 referenceBox = referenceBody.asMatrix();
		// For all objects after the current reference object
		for (size_t j = i+1; j < m_rigid_bodies.size(); j++)
		{
			auto& testBody = m_rigid_bodies[j];
			GamePhysics::Mat4 testBox = testBody.asMatrix();

			CollisionInfo simpletest = checkCollisionSAT(referenceBox, testBox);
			if (simpletest.isValid)
			{
				auto& A = referenceBody;
				auto& B = testBody;
				const auto& collisionPoint = (simpletest.collisionPointWorld);
				std::printf("collision detected at normal: %f, %f, %f\n", simpletest.normalWorld.x, simpletest.normalWorld.y, simpletest.normalWorld.z);
				std::printf("collision point : %f, %f, %f\n", (simpletest.collisionPointWorld).x, (simpletest.collisionPointWorld).y, simpletest.collisionPointWorld.z);
				// Get relative positions of A and B
				const Vec3 relativeA = A.getRelativePositionFromWorld(collisionPoint);
				const Vec3 relativeB = B.getRelativePositionFromWorld(collisionPoint);
				// Get velocity of world point for both A and B
				const Vec3 relativeVelocity = A.getPointVelocity(collisionPoint)- B.getPointVelocity(collisionPoint);
				double impulseJ = helperCalculateImpulse(relativeVelocity, simpletest.normalWorld, relativeA, relativeB, A.m_mass, B.m_mass, A.m_inertiaTensorInverse, B.m_inertiaTensorInverse,1.0);
				
				// Update
				A.m_velocity += (impulseJ * simpletest.normalWorld / A.m_mass);
				B.m_velocity -= (impulseJ * simpletest.normalWorld / B.m_mass);


				A.m_angularMomentum += cross(relativeA, impulseJ * simpletest.normalWorld);
				B.m_angularMomentum -= cross(relativeB, impulseJ * simpletest.normalWorld);
			}
		}
	}

}
