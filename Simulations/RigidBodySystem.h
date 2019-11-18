#pragma once
#include "RigidBodySimulationTypes.h"

class RigidBodySystem
{
public:
	std::vector<Box> m_rigid_bodies;

	
	/**
	* @brief Calculate all constants (center of mass, etc)
	*/
	void initialize();
	void tick(float deltaSeconds);
};
