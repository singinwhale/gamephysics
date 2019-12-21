#include "ParticleIntegratorBase.h"
#include "ParticleSimulationTypes.h"
#include "PhysicsFunctions.h"
#include <map>


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
	std::multimap<ParticleHandle, SpringHandle> particleToSpringsMap;


	for(std::size_t i = 0; i< springs.size(); ++i)
	{
		Spring& spring = springs[i];
		Particle& startParticle = particles[spring.startParticle];
		Particle& endParticle = particles[spring.endParticle];
		spring.force = Physics::CalculateSpringForceBetweenPoints(startParticle.position, endParticle.position, norm(spring.restVector), spring.stiffness);

		// apply the force to the end of the spring
		endParticle.force		+= spring.force;
		// apply the inverse of the force to the second particle because it is pulled in the other direction with the same force (Newton's third law)
		startParticle.force		-= spring.force;
		particleToSpringsMap.emplace(decltype(particleToSpringsMap)::value_type(spring.startParticle, i));
		particleToSpringsMap.emplace(decltype(particleToSpringsMap)::value_type(spring.endParticle, i));
	}


	for(std::size_t i =0;i<particles.size();++i)
	{
		Particle& particle = particles[i];
		auto fromTo = particleToSpringsMap.equal_range(i);
		for (auto it = fromTo.first; it != fromTo.second; ++it)
		{
			Spring& spring = springs[it->second];
			Particle& otherParticle = particles[spring.startParticle == i ? spring.endParticle : spring.startParticle];
			Vec3 springVector = otherParticle.position - particle.position;
			Vec3 restSpringVector = spring.restVector;

			for(auto it2 = it; it2 != fromTo.second; ++it2)
			{
				Spring& otherSpring = springs[it2->second];
				Particle& otherOtherParticle = particles[spring.startParticle == i ? spring.endParticle : spring.startParticle];
				Vec3 otherSpringVector = otherOtherParticle.position - particle.position;
				Vec3 otherRestSpringVector = otherSpring.restVector;

				Vec3 rotAxis = cross(springVector, otherSpringVector);
				Vec3 restRotAxis = cross(restSpringVector, otherRestSpringVector);

				particle.force += cross(springVector, rotAxis - restRotAxis) * angularForce;
			}
		}
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


