#pragma once
#include "ParticleIntegratorBase.h"

/**
 * Integrator that uses the euler method for integrating movement over time
 */
class LeapfrogParticleIntegrator :
	public ParticleIntegratorBase
{

	virtual WorldState GetNextSimulationStep(const WorldState& CurrentWorldState, double deltaSeconds) override;
};

