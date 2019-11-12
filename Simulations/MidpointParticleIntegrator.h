#pragma once
#include "ParticleIntegratorBase.h"
class MidpointParticleIntegrator :
	public ParticleIntegratorBase
{
public:
	virtual WorldState GetNextSimulationStep(const WorldState& CurrentWorldState, double deltaSeconds) override;

private:
	void ComputeForcesOnWorldState(WorldState& worldState);
};

