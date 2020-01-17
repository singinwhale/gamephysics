#include "EulerParticleIntegrator.h"
#include "PhysicsFunctions.h"


WorldState EulerParticleIntegrator::GetNextSimulationStep(const WorldState& CurrentWorldState, double deltaSeconds)
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
		
		const Vec3 acceleration = particle.force / particle.mass + globalAcceleration;

		// integrate position first using the velocity from the last frame
		particle.position = Physics::eulerIntegration(particle.position, particle.velocity, deltaSeconds);
		// only then update the velocity because if you imagine the first frame with no initial velocities:
		// here the only velocity we have is the one that is calculated by the acceleration term. there is no initial velocity
		particle.velocity = Physics::eulerIntegration(particle.velocity, acceleration, deltaSeconds);
	}

	return worldStateCopy;
}
