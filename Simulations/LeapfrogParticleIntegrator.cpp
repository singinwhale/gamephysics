#include "PhysicsFunctions.h"
#include "LeapfrogParticleIntegrator.h"


WorldState LeapfrogParticleIntegrator::GetNextSimulationStep(const WorldState& CurrentWorldState, double deltaSeconds)
{
	WorldState worldStateCopy = CurrentWorldState;
	ClearForces(worldStateCopy);
	
	// first we calculate all the forces affecting the particles so we can 
	// iterate all particles without having to calculate the forces for each one along the way
	CalculateAndStoreSpringForcesForParticles(worldStateCopy.springs, worldStateCopy.particles);
	CalculateAndStoreParticleDampingForces(worldStateCopy.particles);
	StoreGlobalParticleForceForParticles(worldStateCopy.particles);

	for(Particle& particle : worldStateCopy.particles)
	{
		if(particle.isFixed)
			continue;
		
		const Vec3 acceleration = particle.force / particle.mass;
		// integrate velocity first and then immediately use it in position update
		particle.velocity = Physics::eulerIntegration(particle.velocity, acceleration, deltaSeconds);

		// integrate position using the velocity from this frame
		particle.position = Physics::eulerIntegration(particle.position, particle.velocity, deltaSeconds);
	}

	return worldStateCopy;
}
