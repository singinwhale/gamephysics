
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
	TwAddVarRW(DUC->g_pTweakBar, "Friction", TW_TYPE_DOUBLE, &m_params.friction, "");
	TwAddVarRW(DUC->g_pTweakBar, "Angular Damping", TW_TYPE_DOUBLE, &m_params.angularDamping, "");
	TwAddVarRW(DUC->g_pTweakBar, "Linear Damping", TW_TYPE_DOUBLE, &m_params.linearDamping, "");
	TwAddVarRW(DUC->g_pTweakBar, "DebugLifetime", TW_TYPE_UINT32, &m_debugLifetime, "");
	TwAddVarRW(DUC->g_pTweakBar, "Roomscale", TW_TYPE_DOUBLE, &m_roomScale, "");
	TwAddVarRW(DUC->g_pTweakBar, "Default Box Mass", TW_TYPE_DOUBLE, &m_defaultMass, "");
	TwAddVarRW(DUC->g_pTweakBar, "Debug", TW_TYPE_BOOL8, &m_params.showDebugInfo, "");
	TwAddButton(DUC->g_pTweakBar, "Add Box", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		Vec3 size = Vec3(rbss->m_defaultBoxSize[0], rbss->m_defaultBoxSize[1], rbss->m_defaultBoxSize[2]);
		//rbss->addRigidBody(Vec3::ZERO, size, size.x*size.y*size.z);
		std::size_t indexToBox = rbss->addRigidBody(Vec3::ZERO, size * (Vec3::randomDir()/2.0 + Vec3(0.5)), rbss->m_defaultMass);
		rbss->setOrientationOf(indexToBox, Quat::getRandom());
	}, this, "");
	TwAddButton(DUC->g_pTweakBar, "Toggle gravity", [](void* data)
	{
		REINTERPRET_RBSS(rbss, data);
		bool hasGravity = (rbss->m_params.constantAcceleration.squaredDistanceTo(Vec3(0.0)) > 0.0);
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
	double scale = m_roomScale;
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
	m_pRigidBodySystem->m_params = m_params;
	Vec3 size = Vec3(0.1);
	switch(m_iTestCase)
	{
	case 0:
	{		
		const auto index = this->getNumberOfRigidBodies();
		this->addRigidBody(Vec3(0.3, 0.0, 0.0), size, m_defaultMass);
		this->setVelocityOf(index, Vec3(-1.0, 0.0, 0.0));
		this->setOrientationOf(0, Quat(Vec3(1, 0, 0), M_PI / 2.0));
		
		this->addRigidBody(Vec3(0.0, 0.0, 0.0), size, m_defaultMass);

		addWallAndFloorConstraints();
	}
		break;
	case 1:

		this->addRigidBody(Vec3(0.35, 0.35, 0.0), size, m_defaultMass);
		addConstraint(Vec3(0, -0.46, 0), getNormalized(Vec3(-1, 5, 0)));
		addWallAndFloorConstraints();
		break;
	case 2:
	default:
		break;
	}
}

template<class DebugType>
void removeOldDebugObjects(std::vector<DebugType>& InContainer, uint64_t lifetime)
{

	for (std::size_t i = 0; i < InContainer.size(); ++i)
	{
		DebugType& Straight = InContainer[i];
		if (Straight.timestamp + lifetime < g_debugTimer.time)
		{
			continue;
		}
		InContainer.erase(InContainer.begin(), InContainer.begin() + i);
		return;
	}
}

void RigidBodySystemSimulator::drawDebugLines() const
{
	removeOldDebugObjects(m_pRigidBodySystem->m_debugLines, uint64_t(m_debugLifetime));
	for (std::size_t i = 0; i < m_pRigidBodySystem->m_debugLines.size(); ++i)
	{
		m_pRigidBodySystem->m_debugLines[i].Draw(DUC);
	}
}

void RigidBodySystemSimulator::drawDebugRays() const
{
	removeOldDebugObjects(m_pRigidBodySystem->m_debugRays, uint64_t(m_debugLifetime));
	for (std::size_t i = 0; i < m_pRigidBodySystem->m_debugRays.size(); ++i)
	{		
		m_pRigidBodySystem->m_debugRays[i].Draw(DUC);
	}
}

void RigidBodySystemSimulator::drawConstraints() const
{
	for(const PlanarConstraint& constraint: m_pRigidBodySystem->m_constraints)
	{
		DUC->drawLine(constraint.position, Vec3(1, 0, 0), constraint.position + constraint.normal * 0.05, Vec3(0, 1, 0));
	}
}

void RigidBodySystemSimulator::drawBoxes()
{
	for(const Box& box : m_pRigidBodySystem->m_rigid_bodies)
	{
		DUC->drawRigidBody(box.asMatrix());
	}
}

void RigidBodySystemSimulator::drawAngularMomentum()
{
	for (const Box& box : m_pRigidBodySystem->m_rigid_bodies)
	{
		DUC->drawLine(box.m_position, Vec3(0, 0, 1), box.m_position + box.m_angularMomentum * 0.5, Vec3(0, 1, 1));
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3::ZERO, Vec3(1, 1, 1), 0.1, Vec3(.9, .9, .9));
	drawBoxes();
	
	DUC->beginLine();
	drawConstraints();
	drawDebugLines();
	drawDebugRays();
	if(m_params.showDebugInfo)
		drawAngularMomentum();
	DUC->endLine();
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
	g_debugTimer.update();
	m_pRigidBodySystem->m_params = m_params;
	m_pRigidBodySystem->tick(timeStep);
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

std::size_t RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, double mass)
{
	m_pRigidBodySystem->m_rigid_bodies.push_back(Box(position, size, mass));
	return m_pRigidBodySystem->m_rigid_bodies.size() - 1;
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
	m_params.constantAcceleration = force;
}

Vec3 RigidBodySystemSimulator::getConstantForce() const
{
	return m_pRigidBodySystem->m_constantAcceleration;
}

void RigidBodySystemSimulator::addConstraint(Vec3 position, Vec3 normal)
{
	m_pRigidBodySystem->m_constraints.push_back({ position,getNormalized(normal)});
}
