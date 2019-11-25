#pragma once
#include "RigidBodySimulationTypes.h"
#include "collisionDetect.h"

#define USE_VERBOSE 0


class RigidBodySystem
{
public:
	std::vector<Box> m_rigid_bodies;

	RBSSParams m_params;
	
	/// Always-acting force, e.g. gravity
	Vec3 m_constantAcceleration = Vec3(0.0);
	double m_linearDamping = 0.5;
	double m_angularDamping = 0.7;

	// set from outside
	double m_friction;
	
	std::vector<PlanarConstraint> m_constraints;

	std::vector<Line>  m_debugLines;
	std::vector<Ray>  m_debugRays;
	
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
