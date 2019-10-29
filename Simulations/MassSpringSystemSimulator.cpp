#include "MassSpringSystemSimulator.h"
#include "util/FFmpeg.h"
#include "EulerParticleIntegrator.h"
#include "MidpointParticleIntegrator.h"

//Vec3::ZERO is not defined in this compile unit so we create our own definition
const vector3Dim<double>  vector3Dim<double>::ZERO = Vec3(0, 0, 0);
const vector3Dim<float>  vector3Dim<float>::ZERO = vector3Dim(0, 0, 0);

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = EULER;
	/*m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	m_iNumSpheres    = 100;
	m_fSphereSize    = 0.05f;*/
	m_particleIntegrator = std::make_unique<EulerParticleIntegrator>();
}

#define STR(content) #content

const char * MassSpringSystemSimulator::getTestCasesStr(){

	return "EULER,LEAPFROG,MIDPOINT";
}

void MassSpringSystemSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:
		//TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	case 2:break;
	default:break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	cout << "Testcase changed to: ";
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case EULER:
		cout << "Euler !\n";
		m_particleIntegrator = std::make_unique<EulerParticleIntegrator>();
		break;
	case LEAPFROG:
		cout << "Leapfrog (not implemented. using euler instead)!\n";
		m_particleIntegrator = std::make_unique<EulerParticleIntegrator>();
		break;
	case MIDPOINT:
		cout << "Midpoint !\n";
		//m_particleIntegrator = std::make_unique<MidpointParticleIntegrator>();
		break;
	default:
		cout << "Undefined Testcase!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case EULER:
		break;
	case LEAPFROG:
		// not implemented
		break;
	case MIDPOINT:
		cout << "Midpoint !\n";
		break;
	default:
		cout << "Undefined Testcase!\n";
		break;
	}
	m_worldState = m_particleIntegrator->GetNextSimulationStep(m_worldState, timeStep);
}

/*
void MassSpringSystemSimulator::drawSomeRandomObjects()
{
    std::mt19937 eng;
    std::uniform_real_distribution<float> randCol( 0.0f, 1.0f);
    std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
    for (int i=0; i<m_iNumSpheres; i++)
    {
		DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(randCol(eng),randCol(eng), randCol(eng)));
		DUC->drawSphere(Vec3(randPos(eng),randPos(eng),randPos(eng)),Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
    }
}

void MassSpringSystemSimulator::drawMovableTeapot()
{
	DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(0.97,0.86,1));
	DUC->drawTeapot(m_vfMovableObjectPos,m_vfRotate,Vec3(0.5,0.5,0.5));
}

void MassSpringSystemSimulator::drawTriangle()
{
	DUC->DrawTriangleUsingShaders();
}*/

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	// TODO: render scene
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	for(auto& particle : m_worldState.particles)
	{
		particle.mass = mass;
	}
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
	for (auto& spring : m_worldState.springs)
	{
		spring.stiffness = stiffness;
	}
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_particleIntegrator->SetDampingFactor(damping);
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	m_worldState.particles.push_back(Particle(position, Velocity, m_fMass, isFixed));
	return m_worldState.particles.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_worldState.springs.push_back(Spring(ParticleHandle(masspoint1), ParticleHandle(masspoint2), initialLength, m_fStiffness));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_worldState.particles.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_worldState.springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_worldState.particles[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_worldState.particles[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_particleIntegrator->AddGlobalForce(force);
}
