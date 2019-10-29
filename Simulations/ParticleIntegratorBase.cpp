#include "ParticleIntegratorBase.h"
#include "ParticleSimulationTypes.h"


WorldState ParticleIntegratorBase::GetNextSimulationStep(const WorldState& CurrentWorldState, float deltaSeconds)
{
	return CurrentWorldState;

	WorldState NewWorldState = WorldState(CurrentWorldState.particles.size(), CurrentWorldState.springs.size());
	
	std::vector<ParticleUpdateProxy> updateProxyData;
	updat

	for(const Spring& spring : CurrentWorldState.springs)
	{
		
	}
}
