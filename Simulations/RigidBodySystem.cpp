#include "RigidBodySystem.h"
#include "PhysicsFunctions.h"
#include "util/util.h"

void RigidBodySystem::initialize()
{
	for (auto& body : m_rigid_bodies)
	{
		body.calculateInertiaTensor();
	}
}

CollisionInfo RigidBodySystem::getCollisionInfoForPlaneBoxCollision(Box& box, const PlanarConstraint& constraint) const
{
	std::vector<Vec3> trespassingVertexPositions;

	// scope this so we don't accidentally reuse the vertices
	{
		std::vector<Vec3> vertices = box.getVerticesWorldSpace();
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
	size_t counter = 0;
	for (auto& body : m_rigid_bodies)
	{

		VERBOSE(std::cout << "[" << counter <<"]"<< "L" << body.m_angularMomentum << " v:" << body.m_velocity << " @" << body.m_position << std::endl);
		
		// Integrate linear parts
		body.m_position += deltaSeconds * body.m_velocity;

		// Update rotation
		const Vec3 angularVelocity = body.getInertiaTensorInverseWorldSpace() * body.m_angularMomentum;
		
		Quat angularVelocityQuat = Quat(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0);

		body.m_rotation = body.m_rotation + (0.5 * deltaSeconds * angularVelocityQuat) * body.m_rotation;
		body.m_rotation = body.m_rotation.unit();
		++counter;
	}

	// II. Apply forces, torque and damping
	counter = 0;
	for (auto& body : m_rigid_bodies)
	{
		
		// Following code implements slide 14 of gp-lecture05-rigid-bodies-3D
		Vec3 totalForce = m_constantForce;
		Vec3 torque = Vec3(0.0);
		
		for (auto& application : body.m_forceApplications)
		{
			// position x force
			torque += cross(application.first, application.second);
			totalForce += application.second;
		}
		// Clear forces for this frame after we evaluated them
		body.m_forceApplications.clear();

		body.m_velocity += deltaSeconds * (totalForce / body.m_mass);

		// apply damping 
		body.m_velocity += body.m_velocity * -m_linearDamping * deltaSeconds;
		body.m_angularMomentum += body.m_angularMomentum * -m_angularDamping * deltaSeconds;

		VERBOSE(std::cout << "[" << counter << "]" << "Force: " << totalForce << " ---- Torque: " << torque << std::endl);

		// Update angular momentum ...
		body.m_angularMomentum += body.getInertiaTensorInverseWorldSpace() * torque * deltaSeconds;
	}
	
	// III. Detect collisions & apply impluses if neccessary
	for (size_t i = 0; i < m_rigid_bodies.size(); i++)
	{
		auto& referenceBody = m_rigid_bodies[i];
		GamePhysics::Mat4 referenceBox = referenceBody.asMatrix();
		
		// go over all constraints and get all RBs back in line
		for (const PlanarConstraint& constraint : m_constraints)
		{
			CollisionInfo collision_info = getCollisionInfoForPlaneBoxCollision(referenceBody, constraint);
			if (!collision_info.isValid)
				continue;

			const Vec3 relativeCollisionLocationWorldSpace = collision_info.collisionPointWorld - referenceBody.m_position;
			const Vec3 relativeVelocityWorldSpace = referenceBody.getPointVelocityWorldSpace(relativeCollisionLocationWorldSpace);

			const double impulse = Physics::getImpulseForCollision(
				relativeVelocityWorldSpace,
				collision_info.normalWorld,
				relativeCollisionLocationWorldSpace,
				referenceBody.m_mass,
				referenceBody.getInertiaTensorInverseWorldSpace(),
				referenceBody.m_restitution
			);

			referenceBody.m_velocity = referenceBody.m_velocity + collision_info.normalWorld * impulse / referenceBody.m_mass;

			const Vec3 angularMomentumDelta = cross(relativeCollisionLocationWorldSpace, impulse * collision_info.normalWorld);
			referenceBody.m_angularMomentum += angularMomentumDelta;

			// add a friction force parallel to the tangent and opposing the velocity to brake boxes sliding over the constraints
			referenceBody.m_forceApplications.push_back({ relativeCollisionLocationWorldSpace, constraint.projectDirectionOntoPlane(relativeVelocityWorldSpace) * -m_friction });
			// move the object out of the violation so they don't intersect anymore
			referenceBody.m_position += collision_info.depth * collision_info.normalWorld;
			
		}
		
		// we only have to update all bodies after the current one because we apply forces for both
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
			VERBOSE(std::printf("collision point : %f, %f, %f\n", simpletest.collisionPointWorld.x, simpletest.collisionPointWorld.y, simpletest.collisionPointWorld.z));
			
			// Get relative positions of A and B
			const Vec3 collisionPositionRelativeToA = A.m_position - collisionPoint;
			const Vec3 collisionPositionRelativeToB = B.m_position - collisionPoint;
			// Get velocity of world point for both A and B
			const Vec3 relativeVelocityWorldSpace = A.getPointVelocityWorldSpace(collisionPositionRelativeToA) - B.getPointVelocityWorldSpace(collisionPositionRelativeToB);

			double impulseJ = Physics::getImpulseForCollision(
				relativeVelocityWorldSpace,
				simpletest.normalWorld,
				collisionPositionRelativeToA,
				collisionPositionRelativeToB,
				A.m_mass,
				B.m_mass,
				A.getInertiaTensorInverseWorldSpace(),
				B.getInertiaTensorInverseWorldSpace(),
				(A.m_restitution + B.m_restitution) / 2.0
			);
			
			// Update
			A.m_velocity += (impulseJ * simpletest.normalWorld / A.m_mass);
			B.m_velocity -= (impulseJ * simpletest.normalWorld / B.m_mass);

			// move the two objects out of each other so they don't penetrate anymore.
			// use 1.999 instead of 2 so they move a bit farther than perfect separation
			Vec3 penetrationVectorHalf = (simpletest.normalWorld * simpletest.depth)/ (1.9999);
			A.m_position += penetrationVectorHalf;
			B.m_position -= penetrationVectorHalf;
			
			A.m_angularMomentum += cross(collisionPositionRelativeToA, impulseJ * simpletest.normalWorld);
			B.m_angularMomentum += cross(collisionPositionRelativeToB, -impulseJ * simpletest.normalWorld);

			// add friction force
			// TODO
		}

	}

}
