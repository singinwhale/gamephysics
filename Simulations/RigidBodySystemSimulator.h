#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "RigidBodySimulationTypes.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2
#define USE_VERBOSE 0

class RigidBodySystem;

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void addWallAndFloorConstraints();
	void reset();
	void drawDebugLines() const;
	void drawDebugRays() const;
	void drawConstraints() const;
	void drawBoxes();
	void drawAngularMomentum();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	std::size_t addRigidBody(Vec3 position, Vec3 size, double mass = 1.0);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	void setConstantForce(Vec3 force);
	Vec3 getConstantForce() const;

	void addConstraint(Vec3 position, Vec3 normal);

private:
	// Attributes
	std::shared_ptr<RigidBodySystem> m_pRigidBodySystem; 
	RBSSParams m_params;
	
	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	float m_defaultBoxSize[3] = { 0.1f, 0.1f, 0.1f };
	double m_defaultMass = 2;
	double m_roomScale = 0.5;

	// lifetime of debug draws in milliseconds
	uint32_t m_debugLifetime = 1000;
	};
#endif