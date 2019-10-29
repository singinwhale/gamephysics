#pragma once
#include "IParticleIntegrator.h"
struct WorldState;

class ParticleIntegratorBase :
	public IParticleIntegrator
{
public:
	virtual WorldState GetNextSimulationStep(const WorldState& CurrentWorldState, float deltaSeconds) override;

protected:

};

