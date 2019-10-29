#pragma once
#include "ParticleIntegratorBase.h"
class EulerParticleIntegrator :
	public ParticleIntegratorBase
{
public:
	EulerParticleIntegrator();
	~EulerParticleIntegrator();
};

