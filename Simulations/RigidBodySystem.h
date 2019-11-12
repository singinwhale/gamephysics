#pragma once
#include "RigidBodySimulationTypes.h"

class RigidBodySystem
{
public:
	std::vector<Box> m_rigid_bodies;

	void tick(float deltaSeconds);
};
