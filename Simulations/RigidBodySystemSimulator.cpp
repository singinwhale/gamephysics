
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
	TwAddVarRW(DUC->g_pTweakBar, "Friction", TW_TYPE_DOUBLE, &m_friction, "");
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

void RigidBodySystemSimulator::addWallAndFloorConstraints()
{
	double scale = 0.5;
	addConstraint(Vec3(1,0,0)*scale, Vec3(-1,0,0));
	addConstraint(Vec3(-1,0,0)*scale, Vec3(1,0,0));
	addConstraint(Vec3(0,1,0)*scale, Vec3(0,-1,0));
	addConstraint(Vec3(0,-1,0)*scale, Vec3(0,1,0));
	addConstraint(Vec3(0,0,1)*scale, Vec3(0,0,-1));
	addConstraint(Vec3(0,0,-1)*scale, Vec3(0,0,1));
}

void RigidBodySystemSimulator::reset()
{
	m_pRigidBodySystem = std::make_unique<RigidBodySystem>();
	m_pRigidBodySystem->m_constantForce = m_externalForce;
	m_pRigidBodySystem->m_friction = m_friction;
	Vec3 size = Vec3(0.1);
	switch(m_iTestCase)
	{
	case 0:
	{		
		const auto index = this->getNumberOfRigidBodies();
		this->addRigidBody(Vec3(0.3, 0.0, 0.0), size, 0.1);
		this->setVelocityOf(index, Vec3(-1.0, 0.0, 0.0));
		this->setOrientationOf(0, Quat(Vec3(1, 0, 0), M_PI / 2.0));
		
		this->addRigidBody(Vec3(0.0, 0.0, 0.0), size, 0.1);

		addWallAndFloorConstraints();
	}
		break;
	case 1:

		this->addRigidBody(Vec3(0.35, 0.35, 0.0), size, 0.1);
		addConstraint(Vec3(0, -0.46, 0), getNormalized(Vec3(-1, 5, 0)));
		addWallAndFloorConstraints();
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
		DUC->beginLine();
		DUC->drawLine(box.m_position, Vec3(0, 0, 1), box.m_position + box.m_angularMomentum * 0.1, Vec3(0, 1, 1));
		DUC->endLine();
	}
	RigidBodySystem* rbss = m_pRigidBodySystem.get();
	for(const PlanarConstraint& constraint: rbss->m_constraints)
	{
		DUC->beginLine();
		DUC->drawLine(constraint.position, Vec3(1, 0, 0), constraint.position + constraint.normal * 0.1, Vec3(0, 1, 0));
		DUC->endLine();
	}
	DUC->setUpLighting(Vec3(1,0,0), Vec3(1, 1, 1), 0.1, Vec3(.8, 0, 0));
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
	return m_pRigidBodySystem->m_rigid_bodies[i].m_angularMomentum;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pRigidBodySystem->m_rigid_bodies[i].m_forceApplications.push_back(std::make_pair(loc, force));
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, double mass)
{
	m_pRigidBodySystem->m_rigid_bodies.push_back(Box(position, size, mass));
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
	m_externalForce = force;
	m_pRigidBodySystem->m_constantForce = force;
}

Vec3 RigidBodySystemSimulator::getConstantForce() const
{
	return m_pRigidBodySystem->m_constantForce;
}

void RigidBodySystemSimulator::addConstraint(Vec3 position, Vec3 normal)
{
	m_pRigidBodySystem->m_constraints.push_back({ position,getNormalized(normal)});
}
