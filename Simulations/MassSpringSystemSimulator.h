#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "IParticleIntegrator.h"
#include "ParticleSimulationTypes.h"
#include "RigidBodySystem.h"

namespace import {
	class SpringsImporter;
}

class ID3D11DeviceContext;

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


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

	static void TW_CALL demo2(void* simulator);
	static void TW_CALL demo3(void* simulator);
	static void TW_CALL demo5(void* simulator);
	static void TW_CALL handleAddRope(void* simulator);
	static void TW_CALL handleAddIcosphere(void* simulator);
	static void TW_CALL handleAddSuzanne(void* simulator);
	static void TW_CALL handleAddCloth(void* simulator);
	static void TW_CALL handleFixedNet(void* simulator);
	static void TW_CALL handleRandomCube(void* simulator);
	static void TW_CALL handleAddRandomPointButtonClicked(void* simulator);
	static void TW_CALL handleGravityChanged(const void* newValue, void* userData);
	static void TW_CALL twGetGravityCallback(void* targetValue, void* userData);
	static void TW_CALL twSetDampingCallback(const void* targetValue, void* userData);
	static void TW_CALL twGetDampingCallback(void* targetValue, void* userData);

	static void TW_CALL handlePositionChanged(const void* newValue, void* userData);
	static void TW_CALL twGetPositionChangedCallback(void* targetValue, void* userData);
		
	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	Spring& addSpring(int masspoint1, int masspoint2, float initialLength,float stiffness=0.0);
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

	// Rigid body extensions
	void drawBoxes();
	void checkRigidBodyMassSpringIntercollisions(float timestep);

private:
	// Data Attributes
	float m_fMass = 10;
	float m_fStiffness = 2000;
	float m_fDamping = 0;
	int m_iIntegrator = LEAPFROG;

	// UI Attributes
	Vec3 m_externalForce = Vec3::ZERO;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	
	float m_bounceRatio;

	bool m_hasFloor = false;
	bool m_hasBoudaries = false;

	// Simulation State
	WorldState m_worldState;
	std::unique_ptr<IParticleIntegrator> m_particleIntegrators[3];

	std::shared_ptr<RigidBodySystem> m_pRigidBodySystem;
	RBSSParams m_params;

	size_t m_selectedParticle = -1;

	void UseImporter(import::SpringsImporter* importer);
};
#endif