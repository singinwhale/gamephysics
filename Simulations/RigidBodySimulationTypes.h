#pragma once
#include "util/vectorbase.h"
#include "util/quaternion.h"
#include <vector>
#include "util/timer.h"
using namespace GamePhysics;

class DrawingUtilitiesClass;

template<typename T>
T& max(T& a, T& b)
{
	if (a > b)
		return a;
	return b;
}

template<typename T>
T square(T& a)
{
	return a * a;
}

struct RBSSParams
{
	/// Always-acting force, e.g. gravity
	Vec3 constantAcceleration = Vec3(0.0,-9.81,0.0);
	double linearDamping = 0.5;
	double angularDamping = 0.7;

	double friction = 2;
	bool showDebugInfo = false;
};


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
	const Vec3 getPointVelocityWorldSpace(const Vec3 relativePointWorldSpace) const;
	const Vec3 getCollisionImpulse(const Box& otherBox, Vec3 collisionNormal);

	const Mat4d getInertiaTensorInverseWorldSpace() const;

	/** Calculate if bounding spheres of boxes are intersecting */
	const bool haveSphereBVIntersection(const Box& other) const;

	const bool isPointInsideBoundingSphere(const Vec3& point) const;

	const bool hasCollisionWithPoint(const Vec3 point, Vec3& out_collisionPoint) const;
		
	const Vec3 getAngularVelocity() const;
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

extern MuTime g_debugTimer;

struct Straight
{
	Straight() { timestamp = g_debugTimer.time; }
	
	Vec3 colorStart = Vec3(1,0,0);
	Vec3 colorEnd = Vec3(0,1,0);

	unsigned long timestamp;
};

struct Ray : Straight
{
	Ray(Vec3 inStart, Vec3 inDir) : Straight(), start(inStart), direction(inDir) {};
	
	Vec3 start = Vec3::ZERO;
	Vec3 direction = Vec3::ZERO;
	float length = 0.1;
	
	void Draw(DrawingUtilitiesClass* DUC) const;
};

struct Line : Straight
{
	Line(Vec3 inStart, Vec3 inEnd) : Straight(), start(inStart), end(inEnd) { colorStart = colorEnd = Vec3(1, 0, 1); };
	
	Vec3 start = Vec3::ZERO;
	Vec3 end = Vec3::ZERO;

	void Draw(DrawingUtilitiesClass* DUC) const;
};

