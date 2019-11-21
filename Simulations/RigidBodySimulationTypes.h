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
	Quaternion<double> m_rotation = Quaternion<double>(Vec3(1.0, 0.0, 0.0), 0);

	/** extents of the box in worldspace */
	Vec3 m_extents = Vec3(1, 1, 1);

	/** velocity in world space */
	Vec3 m_velocity = Vec3::ZERO;

	/** angular momentum in world space */
	Vec3 m_angularMomentum = Vec3::ZERO;

	/** inverse of inertia tensor in local space */
	Mat4d m_inertiaTensorInverse;

	float m_mass = 1;
	float m_restitution = 0.5;

	Box();
	Box(Vec3 position);
	Box(Vec3 position, Vec3 size);
	Box(Vec3 position, Vec3 size, float mass);
	Mat4 getRotScaleMatrix() const;

	Mat4 asMatrix() const;
	std::vector<Vec3> getVertices() const;
	/// Calculate constants
	void initialize();

	const Vec3 getRelativePositionFromWorld(const Vec3 worldPosition) const;
	const Vec3 getRelativeDirectionFromWorld(Vec3 worldPosition) const;
	const Vec3 getPointVelocity(const Vec3 relativePoint) const;
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
	double distanceToPlane(Vec3 point) const;
};