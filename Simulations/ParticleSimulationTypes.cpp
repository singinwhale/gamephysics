#include "ParticleSimulationTypes.h"

Particle::Particle(Vec3 inPosition, Vec3 iNVelocity, float inMass, bool inIsFixed)
	: position(inPosition)
	, velocity(iNVelocity)
	, mass(inMass)
	, isFixed(inIsFixed)
{
}

Particle::Particle(Vec3 inPosition, Vec3 iNVelocity, float inMass)
	: Particle(inPosition, iNVelocity, inMass)
{
}

Particle::Particle(Vec3 inPosition, Vec3 iNVelocity)
	: Particle(inPosition, iNVelocity, mass)
{
}

Particle::Particle(Vec3 inPosition)
	: Particle(inPosition, Vec3::ZERO)
{
}

Spring::Spring(ParticleHandle start, ParticleHandle end, float length, float inStiffness)
	: startParticle(start)
	, endParticle(end)
	, restLength(length)
	, stiffness(inStiffness)
{
}
