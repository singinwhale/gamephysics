#pragma once
#include "RigidBodySimulationTypes.h"

class RigidBodySystem
{
public:
	std::vector<Box> m_rigid_bodies;
	/// Always-acting force, e.g. gravity
	Vec3 m_constantForce = Vec3(0.0);
	
	/**
	* @brief Calculate all constants (center of mass, etc)
	*/
	void initialize();
	/**
	* @brief Update
	*/
	void tick(float deltaSeconds);

};
