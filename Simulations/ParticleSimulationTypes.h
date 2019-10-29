#pragma once
#include "util/matrixbase.h"
#include <vector>

using namespace GamePhysics;

struct ParticleHandle
{
	size_t Index;
};

struct Particle
{
	Vec3 position = Vec3(0, 0, 0);
	Vec3 velocity = Vec3(0, 0, 0);
	float mass = 0.1;

	Vec3 force = Vec3(0, 0, 0);
};

struct Spring
{
	// particle where the spring starts
	size_t startParticleIndex = -1;
	// particle where the spring ends
	size_t endParticleIndex = -1;

	// rest length of the spring in meters
	float restLength = 1;

	// hook constant for the spring
	float stiffness = 1;

	Vec3 force = Vec3(0, 0, 0);
};


struct WorldState
{
	WorldState(size_t numParticles, size_t numSprings);

	std::vector<Particle> particles;
	std::vector<Spring> springs;
};