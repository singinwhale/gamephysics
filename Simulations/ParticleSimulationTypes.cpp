#include "ParticleSimulationTypes.h"

WorldState::WorldState(size_t numParticles, size_t numSprings)
{
	particles.reserve(numParticles);
	springs.reserve(numSprings);
}
