#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "IParticleIntegrator.h"
#include "ParticleSimulationTypes.h"

class ID3D11DeviceContext;

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

// NumericalMethods.cpp : Defines the entry point for the console application.
//

#include <vector>
#include <iostream>

typedef GamePhysics::Vec3 Vec3;

namespace Roman
{
	class Particle
	{
	public:
		Particle() = default;
		Particle(Vec3 p, Vec3 v, float m, bool fix) : position(p), velocity(v), mass(m), isFixed(fix) {}
		Vec3 position;
		Vec3 velocity;
		float mass;
		bool isFixed;
	};

	class Spring
	{
	public:
		Spring(size_t s, size_t e, float st, float l) : start(s), end(e), stiffness(st), length(l) {}
		size_t start, end;
		float stiffness;
		float length;
	};

	class System
	{
	public:
		std::vector<Particle> m_particles;
		std::vector<Spring> m_springs;
		void printOutParticles()
		{
			for (auto& particle : m_particles)
			{
				std::cout << "Position: [" << particle.position.x << "," << particle.position.y << "," << particle.position.z << "]" << " => ";
				std::cout << "Velocity: [" << particle.velocity.x << "," << particle.velocity.y << "," << particle.velocity.z << "]" << std::endl;
			}
		}
	};

	class Integrator
	{
	public:
		virtual void doStep(float h = 0.1) = 0;
		void setSystem(System* system) { m_system = system; }
		void setGlobalForce(Vec3 force) { m_globalForce = force; }
		const Vec3 getGlobalForce() { return m_globalForce; }
	protected:
		System * m_system;
		Vec3 m_globalForce;
	};

	static Vec3 calculateSpringForce(const Particle& p1, const Particle& p2, float stiffness, float initialLength)
	{
		Vec3 unitVector = (p1.position - p2.position);
		float length = GamePhysics::norm(unitVector);
		if (length < 0.0001)
		{
			length = 0.0001;
		}
		unitVector = unitVector / length;

		float magnitude = -stiffness * (length - initialLength);
		return unitVector * magnitude;
	}

	class Euler : public Integrator
	{
	protected:
		void calculateStep(float h,std::vector<Particle>& input, std::vector<Particle>& output)
		{
			for (size_t index = 0; index < input.size(); index++)
			{
				auto& oldParticle = input[index];
				auto& particle = output[index];
				particle = oldParticle;

				if (!particle.isFixed)
				{
					particle.position += h * input[index].velocity;
				}
				Vec3 totalAcceleration = Vec3(0.0);
				for (auto& spring : this->m_system->m_springs)
				{
					// Skip non related springs
					if (spring.start != index && spring.end != index)
						continue;
					auto& p1 = input[spring.start];
					auto& p2 = input[spring.end];
					Vec3 force = calculateSpringForce(p1, p2, spring.stiffness, spring.length);
					//std::cout << "Force: " << force.x << "," << force.y << "," << force.z << std::endl;
					if (spring.end == index)
					{
						// Inverse force
						force *= -1;
					}
					totalAcceleration += force;
				}

				// Add external force
				float dissipationRatio = 0.2;
				totalAcceleration += getGlobalForce() - input[index].velocity*dissipationRatio;

				//std::cout << "ForceTotal: " << totalAcceleration.x << "," << totalAcceleration.y << "," << totalAcceleration.z << std::endl;
				particle.velocity += h * (totalAcceleration / particle.mass);
			}
		}
	public:
		
		virtual void doStep(float h = 0.1) override
		{
			// Allocate temporary points
			std::vector<Particle> newParticles{ this->m_system->m_particles.size() };
			std::copy(this->m_system->m_particles.begin(), this->m_system->m_particles.end(), newParticles.begin());
			// For each point
			calculateStep(h, this->m_system->m_particles, newParticles);
			// Copy back
			std::copy(newParticles.begin(), newParticles.end(), this->m_system->m_particles.begin());
		}
	};

	class Midpoint : public Euler
	{
	public:
		virtual void doStep(float h = 0.1) override
		{
			// Allocate temporary points
			std::vector<Particle> newParticles{ this->m_system->m_particles.size() };
			std::copy(this->m_system->m_particles.begin(), this->m_system->m_particles.end(), newParticles.begin());
			// For each point
			calculateStep(h*0.5, this->m_system->m_particles, newParticles);
			calculateStep(h, newParticles, this->m_system->m_particles);
		}
	};

}

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	// Calculate the ray direction from camera's origin, going through point (px,py) at the screen
	Vec3 screenToRay(const float px, const float py);
	Vec3 getCameraPosition();
	Vec3 getScreenSize();
	// Project world-based point to position at the screen
	Vec3 pointToScreen(const Vec3 point3D);

	size_t findClosesPoint(const WorldState& world, float px, float py);
	void renderWorldParticles(const WorldState& world);

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Simulation State
	WorldState m_worldState;

	size_t m_selectedParticle = -1;

	std::unique_ptr<IParticleIntegrator> m_particleIntegrator;

	Roman::System m_romanSys;
	Roman::Midpoint m_romanInt;
};
#endif