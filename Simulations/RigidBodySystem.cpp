#include "RigidBodySystem.h"
#include "PhysicsFunctions.h"
#include "util/util.h"

#ifdef VERBOSE
#undef VERBOSE
#define VERBOSE(code) if(m_params.showDebugInfo){ code }
#endif


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
			const Vec3 constraintToTargetLine = constraint.projectOntoPlane(vertex);
			const double distanceToPlane = constraint.distanceToPlane(vertex);

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

		VERBOSE(std::cout << "[" << counter << "]" << "L" << body.m_angularMomentum << " v:" << body.m_velocity << " @" << body.m_position << std::endl;)
		
		// Integrate linear parts
		body.m_position += deltaSeconds * body.m_velocity;

		// Update rotation
		const Vec3 angularVelocity = body.getInertiaTensorInverseWorldSpace() * body.m_angularMomentum;
		Quat angularVelocityQuat = Quat(getNormalized(angularVelocity), norm(angularVelocity));

		VERBOSE(
			m_debugRays.emplace_back(Ray(body.m_position + Vec3(0, 0.2, 0), (Quat::slerp(body.m_rotation,angularVelocityQuat,1.0).getRotMat().transformVector(Vec3(1,0,0)))));
			m_debugRays.emplace_back(Ray(body.m_position + Vec3(0, 0.2, 0), (Quat::slerp(body.m_rotation,angularVelocityQuat,.7).getRotMat().transformVector(Vec3(1,0,0)))));
			m_debugRays.emplace_back(Ray(body.m_position + Vec3(0, 0.2, 0), (Quat::slerp(body.m_rotation,angularVelocityQuat,.33).getRotMat().transformVector(Vec3(1,0,0)))));
			m_debugRays.emplace_back(Ray(body.m_position + Vec3(0, 0.2, 0), (Quat::slerp(body.m_rotation,angularVelocityQuat,0).getRotMat().transformVector(Vec3(1,0,0)))));
		)
		
		body.m_rotation = body.m_rotation + angularVelocityQuat * deltaSeconds;
		body.m_rotation = body.m_rotation.unit();
		++counter;
	}

	// II. Apply forces, torque and damping
	counter = 0;
	for (auto& body : m_rigid_bodies)
	{
		
		// Following code implements slide 14 of gp-lecture05-rigid-bodies-3D
		Vec3 totalForce = Vec3::ZERO;
		Vec3 torque = Vec3(0.0);
		
		for (auto& application : body.m_forceApplications)
		{
			// position x force
			torque += cross(application.first, application.second);
			totalForce += application.second;
		}
		// Clear forces for this frame after we evaluated them
		body.m_forceApplications.clear();

		body.m_velocity += deltaSeconds * (totalForce / body.m_mass) + m_params.constantAcceleration*deltaSeconds;

		// apply damping 
		body.m_velocity += body.m_velocity * -m_params.linearDamping * deltaSeconds;
		body.m_angularMomentum += body.m_angularMomentum * -m_params.angularDamping * deltaSeconds;

		VERBOSE(std::cout << "[" << counter << "]" << "Force: " << totalForce << " ---- Torque: " << torque << std::endl;)

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

			// move the object out of the violation so they don't intersect anymore
			referenceBody.m_position += collision_info.depth * collision_info.normalWorld * 1.0001;
			
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



			VERBOSE(
				m_debugLines.emplace_back(Line(referenceBody.m_position, collision_info.collisionPointWorld));
				m_debugRays.emplace_back(Ray(collision_info.collisionPointWorld, collision_info.normalWorld));
				m_debugRays.back().length *= impulse;
			)

			referenceBody.m_velocity = referenceBody.m_velocity + collision_info.normalWorld * impulse / referenceBody.m_mass;

			const Vec3 angularMomentumDelta = cross(relativeCollisionLocationWorldSpace, impulse * collision_info.normalWorld);
			referenceBody.m_angularMomentum += angularMomentumDelta;

			// add a friction force parallel to the tangent and opposing the velocity to brake boxes sliding over the constraints
			referenceBody.m_forceApplications.push_back({ relativeCollisionLocationWorldSpace, constraint.projectDirectionOntoPlane(relativeVelocityWorldSpace) * -m_params.friction });
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
			
			const Vec3& collisionPoint = (simpletest.collisionPointWorld);
			
			VERBOSE(
				std::printf("collision detected at normal: %f, %f, %f\n", simpletest.normalWorld.x, simpletest.normalWorld.y, simpletest.normalWorld.z);
				std::printf("collision point : %f, %f, %f\n", simpletest.collisionPointWorld.x, simpletest.collisionPointWorld.y, simpletest.collisionPointWorld.z);
			)


			// move the two objects out of each other so they don't penetrate anymore.
			// use 1.999 instead of 2 so they move a bit farther than perfect separation
			Vec3 penetrationVectorHalf = (simpletest.normalWorld * simpletest.depth) / (1.9999);
			A.m_position += penetrationVectorHalf;
			B.m_position -= penetrationVectorHalf;
			
			// Get relative positions of A and B
			const Vec3 collisionPositionRelativeToA = collisionPoint - A.m_position;
			const Vec3 collisionPositionRelativeToB =  collisionPoint - B.m_position;
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


			VERBOSE(
				m_debugLines.emplace_back(Line(A.m_position + Vec3(0, 0.2, 0), collisionPoint + Vec3(0, 0.2, 0)));
				m_debugLines.emplace_back(Line(B.m_position + Vec3(0, 0.2, 0), collisionPoint + Vec3(0, 0.2, 0)));
				m_debugRays.emplace_back(Ray(collisionPoint + Vec3(0, 0.2, 0), simpletest.normalWorld));
				m_debugRays.back().length *= impulseJ;
			)
			// Update
			A.m_velocity += (impulseJ * simpletest.normalWorld / A.m_mass);
			B.m_velocity -= (impulseJ * simpletest.normalWorld / B.m_mass);
			
			A.m_angularMomentum += cross(collisionPositionRelativeToA, impulseJ * simpletest.normalWorld);
			B.m_angularMomentum += cross(collisionPositionRelativeToB, -impulseJ * simpletest.normalWorld);

			// add friction force
			const Vec3 tangentialVelocity = relativeVelocityWorldSpace- dot(simpletest.normalWorld, relativeVelocityWorldSpace);
			A.m_forceApplications.push_back({collisionPoint, tangentialVelocity * -m_params.friction});
			B.m_forceApplications.push_back({collisionPoint, tangentialVelocity * m_params.friction});
		}

	}

}
