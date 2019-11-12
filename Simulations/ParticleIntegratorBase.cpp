#include "ParticleIntegratorBase.h"
#include "ParticleSimulationTypes.h"
#include "PhysicsFunctions.h"


WorldState ParticleIntegratorBase::GetNextSimulationStep(const WorldState& CurrentWorldState, double deltaSeconds)
{
	return CurrentWorldState;
}

void ParticleIntegratorBase::AddGlobalForce(Vec3 force)
{
	globalForce += force;
}

void ParticleIntegratorBase::ResetGlobalForces()
{
	globalForce = Vec3::ZERO;
}

void ParticleIntegratorBase::SetDampingFactor(float dampingFactor)
{
	damping = dampingFactor;
}

void ParticleIntegratorBase::ClearForces(WorldState& state) const
{
	for(Particle& particle : state.particles)
	{
		particle.force = Vec3::ZERO;
	}
	for(Spring& spring : state.springs)
	{
		spring.force = Vec3::ZERO;
	}
}

void ParticleIntegratorBase::CalculateAndStoreSpringForcesForParticles(std::vector<Spring>& springs, std::vector<Particle>& particles) const
{
	for(Spring& spring : springs)
	{
		Particle& startParticle = particles[spring.startParticle];
		Particle& endParticle = particles[spring.endParticle];
		spring.force = Physics::CalculateSpringForceBetweenPoints(startParticle.position, endParticle.position, spring.restLength, spring.stiffness);

		// apply the force to the end of the spring
		endParticle.force		+= spring.force;
		// apply the inverse of the force to the second particle because it is pulled in the other direction with the same force (Newton's third law)
		startParticle.force		-= spring.force;
	}
}

void ParticleIntegratorBase::StoreGlobalParticleForceForParticles(std::vector<Particle>& particles) const
{
	for (Particle& particle : particles)
	{
		particle.force += globalForce;
	}
}

void ParticleIntegratorBase::CalculateAndStoreParticleDampingForces(std::vector<Particle>& particles) const
{
	for (Particle& particle : particles)
	{
		particle.force += Physics::CalculateDampingForce(particle.velocity, damping);
	}
}


