#pragma once
#include "util/matrixbase.h"
using namespace GamePhysics;

struct WorldState;


class IParticleIntegrator
{
public:
	/**
	 * @param CurrentWorldState the current state of the simulation
	 * @param deltaSeconds the time which should pass in the world
	 * @return the new world state after deltaSeconds time has passed in the world
	 */
	virtual WorldState GetNextSimulationStep(const WorldState& CurrentWorldState, float deltaSeconds) = 0;

	/**
	 * adds a vector to the global forces which will be applied to every particle during the simulation
	 */
	virtual void AddGlobalForce(Vec3 force) = 0;

	/**
	 * Resets the global forces to (0,0,0)
	 */
	virtual void ResetGlobalForces() = 0;

	virtual void SetDampingFactor(float dampingFactor) = 0;
};

