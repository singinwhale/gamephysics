#pragma once
#include "RigidBodySimulationTypes.h"
#include "collisionDetect.h"

#define USE_VERBOSE 0

class RigidBodySystem
{
public:
	std::vector<Box> m_rigid_bodies;
	/// Always-acting force, e.g. gravity
	Vec3 m_constantForce = Vec3(0.0);
	double m_linearDamping = 0.5;
	double m_angularDamping = 0.1;
	double m_friction = 15;
	
	std::vector<PlanarConstraint> m_constraints;
	
	/**
	* @brief Calculate all constants (center of mass, etc)
	*/
	void initialize();
	
	/**
	* @brief Update
	*/
	void tick(float deltaSeconds);

	CollisionInfo getCollisionInfoForPlaneBoxCollision(Box& box, const PlanarConstraint& constraint) const;
};
