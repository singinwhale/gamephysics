#include "RigidBodySystem.h"
#include "PhysicsFunctions.h"
#include "util/util.h"

void RigidBodySystem::initialize()
{
	for (auto& body : m_rigid_bodies)
	{
		body.initialize();
	}
}

CollisionInfo RigidBodySystem::getCollisionInfoForPlaneBoxCollision(Box& box, const PlanarConstraint& constraint) const
{
	std::vector<Vec3> trespassingVertexPositions;

	// scope this so we don't accidentally reuse the vertices
	{
		std::vector<Vec3> vertices = box.getVertices();
		for (const Vec3& vertex : vertices)
		{
			const Vec3 constraintToTargetLine = (vertex - constraint.position);
			const double distanceToPlane = dot(constraintToTargetLine, constraint.normal);

			if (distanceToPlane < 0)
			{
				trespassingVertexPositions.push_back(vertex);
			}
		}
	}
	CollisionInfo collision_info;
	collision_info.isValid = false;

	if (trespassingVertexPositions.empty())
		return collision_info;
	
			
	// find average position which is where we will apply the force to
	Vec3 averageTrespassingPosition = Vec3::ZERO;
	for (const Vec3& vertex : trespassingVertexPositions)
	{
		averageTrespassingPosition += vertex;
	}
	averageTrespassingPosition /= trespassingVertexPositions.size();

	collision_info.collisionPointWorld = constraint.projectOntoPlane(averageTrespassingPosition);
	collision_info.depth = -constraint.distanceToPlane(averageTrespassingPosition);
	collision_info.isValid = true;
	collision_info.normalWorld = constraint.normal;

	return collision_info;
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

		VERBOSE(std::cout << "Force: " << totalForce << " ---- Torque: " << torque << std::endl);

		// Integrate linear parts
		body.m_position += deltaSeconds * body.m_velocity;
		body.m_velocity += deltaSeconds * (totalForce/body.m_mass);

		
		VERBOSE(std::cout << "[Mass: " << body.m_mass << "] Position: " << body.m_position << " - s [" << body.m_velocity << "]" << std::endl);
		
		// Angular velocity
		const auto& w = body.m_angularMomentum;

		// Update angular rotation
		Quaternion<double>& derivative = Quaternion<double>(getNormalized(w),norm(w));
		body.m_rotation += ((deltaSeconds / 2.0)*derivative)*body.m_rotation;
		body.m_rotation = body.m_rotation.unit();
	
		// Update angular momentum ...
		body.m_angularMomentum = body.m_angularMomentum + body.getInertiaTensorInverseWorldSpace() * deltaSeconds * torque;
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
			if (!simpletest.isValid)
				continue;
			auto& A = referenceBody;
			auto& B = testBody;
			const auto& collisionPoint = (simpletest.collisionPointWorld);
			VERBOSE(std::printf("collision detected at normal: %f, %f, %f\n", simpletest.normalWorld.x, simpletest.normalWorld.y, simpletest.normalWorld.z));
			VERBOSE(std::printf("collision point : %f, %f, %f\n", (simpletest.collisionPointWorld).x, (simpletest.collisionPointWorld).y, simpletest.collisionPointWorld.z));
			// Get relative positions of A and B
			const Vec3 relativeA = A.getRelativePositionFromWorld(collisionPoint);
			const Vec3 relativeB = B.getRelativePositionFromWorld(collisionPoint);
			// Get velocity of world point for both A and B
			const Vec3 relativeVelocity = A.getPointVelocity(collisionPoint)- B.getPointVelocity(collisionPoint);

			double impulseJ = Physics::getImpulseForCollision(
				relativeVelocity, 
				simpletest.normalWorld, 
				relativeA, 
				relativeB, 
				A.m_mass, 
				B.m_mass,
				A.m_inertiaTensorInverse, 
				B.m_inertiaTensorInverse,
				1.0
			);
			
			// Update
			A.m_velocity += (impulseJ * simpletest.normalWorld / A.m_mass);
			B.m_velocity -= (impulseJ * simpletest.normalWorld / B.m_mass);

			// move the two objects out of each other so they don't penetrate anymore.
			// use 1.999 instead of 2 so they move a bit farther than perfect separation
			Vec3 penetrationVectorHalf = (simpletest.normalWorld * simpletest.depth)/ (1.9999);
			A.m_position += penetrationVectorHalf;
			B.m_position -= penetrationVectorHalf;
			
			A.m_angularMomentum += cross(relativeA, impulseJ * simpletest.normalWorld);
			B.m_angularMomentum -= cross(relativeB, impulseJ * simpletest.normalWorld);	
		}


		// go over all constraints and get all RBs back in line
		for (const PlanarConstraint& constraint : m_constraints)
		{
			CollisionInfo collision_info = getCollisionInfoForPlaneBoxCollision(referenceBody, constraint);

			if (!collision_info.isValid)
				continue;
			
			const Vec3 relativeVelocityWorldSpace = referenceBody.m_velocity;			
			const Vec3 relativeCollisionLocationWorldSpace = collision_info.collisionPointWorld - referenceBody.m_position;
			
			const double impulse = Physics::getImpulseForCollision(
				relativeVelocityWorldSpace,
				collision_info.normalWorld,
				relativeCollisionLocationWorldSpace, 
				referenceBody.m_mass, 
				referenceBody.getInertiaTensorInverseWorldSpace(), 
				referenceBody.m_restitution
			);
			
			//the impulse is taken abs because we always want the force to push the object away from the constraint
			referenceBody.m_velocity = referenceBody.m_velocity + collision_info.normalWorld * impulse/referenceBody.m_mass;

			// move the object out of the violation
			referenceBody.m_position += collision_info.depth * collision_info.normalWorld;
			referenceBody.m_angularMomentum = referenceBody.m_angularMomentum + cross(referenceBody.m_angularMomentum,impulse * collision_info.normalWorld);
		}
	}

}
