#include "EulerParticleIntegrator.h"


WorldState EulerParticleIntegrator::GetNextSimulationStep(const WorldState& CurrentWorldState, float deltaSeconds)
{
	WorldState WorldStateCopy = CurrentWorldState;

	// first we calculate all the forces affecting the particles so we can 
	// iterate all particles without having to calculate the forces for each one along the way
	CalculateAndStoreSpringForcesForParticles(WorldStateCopy.springs, WorldStateCopy.particles);
	CalculateAndStoreParticleDampingForces(WorldStateCopy.particles);
	StoreGlobalParticleForceForParticles(WorldStateCopy.particles);

	for(Particle& particle : WorldStateCopy.particles)
	{
		const Vec3 acceleration = particle.force / particle.mass;

		// integrate position first using the velocity from the last frame
		particle.velocity = particle.velocity + acceleration * deltaSeconds;
		// only then update the velocity because if you imagine the first frame with no initial velocities:
		// here the only velocity we have is the one that is calculated by the acceleration term. there is no initial velocity
		particle.position = particle.position + particle.velocity * deltaSeconds + 0.5*acceleration*deltaSeconds*deltaSeconds;
	}

	return WorldStateCopy;
}
