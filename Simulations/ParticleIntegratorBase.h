#pragma once
#include "IParticleIntegrator.h"
#include "ParticleSimulationTypes.h"

class ParticleIntegratorBase :
	public IParticleIntegrator
{
public:
	virtual WorldState GetNextSimulationStep(const WorldState& CurrentWorldState, double deltaSeconds) override;

	virtual void AddGlobalForce(Vec3 force) override;
	virtual void ResetGlobalForces() override;

	virtual void SetDampingFactor(float dampingFactor) override;
protected:
	/** Sum of all forces that should be applied to all particles every tick */
	Vec3 globalForce = Vec3::ZERO;

	float damping = 0;

	float angularForce = 2;

	/** Resets any residual forces in the given state that might remain from a previous simulation step */
	void ClearForces(WorldState& state) const;

	void CalculateAndStoreSpringForcesForParticles(std::vector<Spring>& springs, std::vector<Particle>& particles) const;

	void StoreGlobalParticleForceForParticles(std::vector<Particle>& particles) const;

	void CalculateAndStoreParticleDampingForces(std::vector<Particle>& particles) const;
};

