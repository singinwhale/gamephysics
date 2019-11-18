
#include "RigidBodySystemSimulator.h"
#include "RigidBodySimulationTypes.h"
#include "RigidBodySystem.h"

#define REINTERPRET_RBSS(VarName, rigidBodySystemSimulatorPtr) RigidBodySystemSimulator* VarName = reinterpret_cast<RigidBodySystemSimulator*>(rigidBodySystemSimulatorPtr)

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = TESTCASEUSEDTORUNTEST;
	reset();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	
	TwAddVarRW(DUC->g_pTweakBar, "Box Size", TW_TYPE_DIR3F, &m_defaultBoxSize[0], "");
	TwAddButton(DUC->g_pTweakBar, "Add Box", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		Vec3 size = Vec3(rbss->m_defaultBoxSize[0], rbss->m_defaultBoxSize[1], rbss->m_defaultBoxSize[2]);
		//rbss->addRigidBody(Vec3::ZERO, size, size.x*size.y*size.z);
		rbss->addRigidBody(Vec3::ZERO, size, 1.0);
	}, this, "");
	TwAddButton(DUC->g_pTweakBar, "Toggle gravity", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		bool hasGravity = (rbss->getConstantForce().squaredDistanceTo(Vec3(0.0)) > 0.0);
		rbss->setConstantForce(hasGravity ? Vec3(0.0) : Vec3(0.0, -9.81, 0.0));
	}, this, "");

	TwAddButton(DUC->g_pTweakBar, "Add left force to last object", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		const auto count = rbss->getNumberOfRigidBodies();
		rbss->applyForceOnBody(count - 1, Vec3(1.0, 0.0, 0.0), Vec3(0.0, 30.0, 0.0));
	}, this, "");

	TwAddButton(DUC->g_pTweakBar, "Add right force to last object", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		const auto count = rbss->getNumberOfRigidBodies();
		rbss->applyForceOnBody(count - 1, Vec3(-1.0, 0.0, 0.0), Vec3(0.0, 30.0, 0.0));
	}, this, "");
}

void RigidBodySystemSimulator::reset()
{
	m_pRigidBodySystem = std::make_unique<RigidBodySystem>();
	switch(m_iTestCase)
	{
	case 0:
		break;
	case 1:
		break;
	case 2:
	default:
		break;
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3::ZERO, Vec3(1, 1, 1), 0.1, Vec3(.9, .9, .9));
	for(const Box& box : m_pRigidBodySystem->m_rigid_bodies)
	{
		DUC->drawRigidBody(box.asMatrix());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//TODO
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	//TODO
	this->m_pRigidBodySystem->tick(timeStep);
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_oldtrackmouse = m_oldtrackmouse;
	m_trackmouse = { x,y };
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_mouse = { x,y };
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem->m_rigid_bodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigid_bodies[i].m_position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigid_bodies[i].m_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigid_bodies[i].m_angularvelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pRigidBodySystem->m_rigid_bodies[i].m_forceApplications.push_back(std::make_pair(loc, force));
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_pRigidBodySystem->m_rigid_bodies.push_back(Box(position, size, mass));
	m_pRigidBodySystem->m_rigid_bodies.back().m_angularMomentum.x = 1.0;
	m_pRigidBodySystem->m_rigid_bodies.back().m_angularMomentum.y = 2.0;
	m_pRigidBodySystem->m_rigid_bodies.back().m_angularMomentum.z = 3.0;
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem->m_rigid_bodies[i].m_rotation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_pRigidBodySystem->m_rigid_bodies[i].m_velocity = velocity;
}

void RigidBodySystemSimulator::setConstantForce(Vec3 force)
{
	m_pRigidBodySystem->m_constantForce = force;
}

Vec3 RigidBodySystemSimulator::getConstantForce() const
{
	return m_pRigidBodySystem->m_constantForce;
}
