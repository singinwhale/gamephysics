#include "MidpointParticleIntegrator.h"
#include "PhysicsFunctions.h"

void MidpointParticleIntegrator::ComputeForcesOnWorldState(WorldState& worldState)
{
	CalculateAndStoreSpringForcesForParticles(worldState.springs, worldState.particles);
	CalculateAndStoreParticleDampingForces(worldState.particles);
	StoreGlobalParticleForceForParticles(worldState.particles);
}

WorldState MidpointParticleIntegrator::GetNextSimulationStep(const WorldState& CurrentWorldState, double deltaSeconds)
{
	WorldState midPointState = CurrentWorldState;

	// compute forces at local position so we can do the half step using the local forces
	ClearForces(midPointState);
	ComputeForcesOnWorldState(midPointState);

	// compute midpoint euler integration for half timestep
	for (Particle& particle : midPointState.particles)
	{
		const Vec3 acceleration = particle.force / particle.mass;
		particle.position = Physics::eulerIntegration(particle.position, particle.velocity, deltaSeconds/2);
		particle.velocity = Physics::eulerIntegration(particle.velocity, acceleration, deltaSeconds/2);
	}

	// recompute forces for the midpoint step after the halfstep so we have the correct forces at the midpoint
	ClearForces(midPointState);
	ComputeForcesOnWorldState(midPointState);

	// working copy of the current state that we will return as the new state
	WorldState localState = CurrentWorldState;

	// now apply the midpoint tangents to the local particles
	for (size_t i = 0; i < localState.particles.size(); ++i)
	{
		Particle& localParticle = localState.particles[i];
		const Particle& midpointParticle = midPointState.particles[i];

		const Vec3 midpointAcceleration = midpointParticle.force / midpointParticle.mass;

		// velocity is the slope of the position function so we take the velocity at the midpoint (not the force)
		// and make a full step with that
		localParticle.position = Physics::eulerIntegration(localParticle.position, midpointParticle.velocity, deltaSeconds);

		// acceleration is the slope of the velocity so we take the acceleration at the midpoint 
		// and use that to do an euler step using the data there
		localParticle.velocity = Physics::eulerIntegration(localParticle.velocity, midpointAcceleration, deltaSeconds);

		// copy over the force from the midpoint state so we can visualize it. not required for correct computation
		localParticle.force = midpointParticle.force;
	}

	return localState;
}
