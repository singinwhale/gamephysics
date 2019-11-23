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
	// <position, force> force impulses in current frame
	std::vector<std::pair<Vec3, Vec3>> m_forceApplications;

	/** local to world position */
	Vec3 m_position = Vec3::ZERO;

	/** local to world rotation */
	Quat m_rotation = Quat(Vec3(1.0, 0.0, 0.0), 0);

	/** extents of the box in worldspace */
	Vec3 m_extents = Vec3(1, 1, 1);

	/** velocity in world space */
	Vec3 m_velocity = Vec3::ZERO;

	/** angular momentum in world space */
	Vec3 m_angularMomentum = Vec3::ZERO;

	/** inverse of inertia tensor in local space */
	Mat4d m_inertiaTensorInverse;

	double m_mass = 1;
	double m_restitution = 0.6;

	Box();
	Box(Vec3 position);
	Box(Vec3 position, Vec3 size);
	Box(Vec3 position, Vec3 size, double mass);
	Mat4 getRotScaleMatrix() const;

	Mat4 asMatrix() const;
	std::vector<Vec3> getVerticesWorldSpace() const;
	/// Calculate constants
	void calculateInertiaTensor();

	const Vec3 getRelativePositionFromWorld(const Vec3 worldPosition) const;
	const Vec3 getRelativeDirectionFromWorld(Vec3 worldPosition) const;
	const Vec3 getPointVelocityWorldSpace(const Vec3 relativePointWorldSpace) const;
	const Vec3 getCollisionImpulse(const Box& otherBox, Vec3 collisionNormal);

	const Mat4d getInertiaTensorInverseWorldSpace() const;
};

struct PlanarConstraint
{
	/** position of the plane in worldspace */
	Vec3 position = Vec3::ZERO;

	/** normal of the plane in worldspace*/
	Vec3 normal = Vec3::ZERO;

	Vec3 projectOntoPlane(Vec3 point) const;
	Vec3 projectDirectionOntoPlane(Vec3 direction) const;
	double distanceToPlane(Vec3 point) const;
};