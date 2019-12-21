#include "ParticleSimulationTypes.h"

Particle::Particle(Vec3 inPosition, Vec3 iNVelocity, double inMass, bool inIsFixed)
	: position(inPosition)
	, velocity(iNVelocity)
	, mass(inMass)
	, isFixed(inIsFixed)
{
}

Particle::Particle(Vec3 inPosition, Vec3 iNVelocity, double inMass)
	: Particle(inPosition, iNVelocity, inMass, false)
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

Spring::Spring(ParticleHandle start, ParticleHandle end, Vec3 lengthVector, double inStiffness)
	: startParticle(start)
	, endParticle(end)
	, restVector(lengthVector)
	, stiffness(inStiffness)
{
}

bool operator==(const Spring& A, const Spring& B)
{
	if (A.startParticle == B.startParticle && A.endParticle == B.endParticle)
		return true;
	if (A.startParticle == B.endParticle && A.endParticle == B.endParticle)
		return true;
	return false;
}

bool operator<(const Spring& A, const Spring& B)
{
	if (A.startParticle < B.startParticle && A.endParticle < B.endParticle)
		return true;
	if (A.startParticle < B.endParticle && A.endParticle < B.endParticle)
		return true;
	return false;
}
