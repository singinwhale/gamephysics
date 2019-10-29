#pragma once
#include "util/matrixbase.h"
#include <vector>

using namespace GamePhysics;

typedef size_t ParticleHandle;

struct Particle
{
	Vec3 position = Vec3::ZERO;
	Vec3 velocity = Vec3::ZERO;
	float mass = 0.1;
	bool isFixed = false;

	// only used during update to accumulate forces
	Vec3 force = Vec3::ZERO;

	// Constructors
	Particle(Vec3 inPosition, Vec3 iNVelocity, float inMass, bool inIsFixed);
	Particle(Vec3 inPosition, Vec3 iNVelocity, float inMass);
	Particle(Vec3 inPosition, Vec3 iNVelocity);
	Particle(Vec3 inPosition);
};

struct Spring
{
	// particle where the spring starts
	ParticleHandle startParticle = -1;
	// particle where the spring ends
	ParticleHandle endParticle = -1;

	// rest length of the spring in meters
	float restLength = 1;

	// hook constant for the spring
	float stiffness = 1;

	// only used during update
	Vec3 force = Vec3::ZERO;

	Spring(ParticleHandle start, ParticleHandle end, float length, float inStiffness);
};


struct WorldState
{
	std::vector<Particle> particles;
	std::vector<Spring> springs;
};