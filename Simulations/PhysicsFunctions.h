#pragma once
#include "util/matrixbase.h"

using namespace GamePhysics;

namespace Physics
{

	inline Vec3 CalculateSpringForceBetweenPoints(Vec3 pointA, Vec3 pointB, double springRestlength, double springConstant)
	{
		const Vec3 springVector = pointB - pointA;
		double springLength = norm(springVector);
		// Hack: if points are infinitely close together, then force is zero (to prevent infty)
		if (springLength < std::numeric_limits<double>::epsilon())
		{
			return Vec3();
		}
		const double forceMagnitude = -springConstant * (springLength - springRestlength);
		return (springVector / springLength) * forceMagnitude;
	}

	inline Vec3 CalculateDampingForce(Vec3 velocity, double dampingFactor)
	{
		return -velocity * dampingFactor;
	}

	inline Vec3 eulerIntegration(Vec3 value, Vec3 slope, double deltaSeconds)
	{
		return value + slope * deltaSeconds;
	}
	
	/** returns the magnitude of the impulse resulting from the collision between two rigid bodies */
	inline double getImpulseForCollision(Vec3 relativeVelocity, Vec3 normal, Vec3 positionA, Vec3 positionB, float massA, float massB, Mat4 inertiaA, Mat4 inertiaB, float restitution = 1.0)
	{
		const auto nominator = -(1 + restitution) * (dot(normal, relativeVelocity));
		const auto inverseMasses = 1.0 / massA + 1.0 / massB;
		const auto impulse3dA = inertiaA * (cross(positionA, normal));
		const auto impulse3dB = inertiaB * (cross(positionB, normal));
		const Vec3 addedImpulseAxes = cross(impulse3dA,positionA) + cross(impulse3dB,positionB);
		const auto denominator = inverseMasses + dot(addedImpulseAxes, normal);
		return nominator / denominator;
	}

	/** returns the magnitude of the impulse for a Collision between an immovable object and a rigid body */
	inline double getImpulseForCollision(Vec3 relativeVelocity, Vec3 normal, Vec3 position, double mass, Mat4 inertiaTensorInverse, double restitution = 1.0)
	{
		const auto nominator = -(1.0 + restitution) * (dot(normal, relativeVelocity));
		const auto inverseMass = 1.0 / mass;
		const auto impulse3d = inertiaTensorInverse.transformVector(cross(position, normal));
		const auto denominator = inverseMass + dot(cross(impulse3d, position), normal);
		return nominator / denominator;
	}
}