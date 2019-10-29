#pragma once
#include "ParticleIntegratorBase.h"
class MidpointParticleIntegrator :
	public ParticleIntegratorBase
{
public:
	MidpointParticleIntegrator();
	~MidpointParticleIntegrator();
};

