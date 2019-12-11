#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(const size_t x, const size_t y) {
	// Allocated & fill grid with 0.0
	this->m_grid = std::vector<cell_t>(x*y, 0.0);
	this->m_width = x;
	this->m_height = y;
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
	this->m_status.m_grid = std::make_unique<Grid>(Grid(3, 3));
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarCB(DUC->g_pTweakBar, "Size M", TW_TYPE_UINT32, [] (const void* targetValue, void* userData) 
	{
		const uint32_t m = *reinterpret_cast<const uint32_t*>(targetValue);
		auto ds = reinterpret_cast<DiffusionSimulator*>(userData);
		ds->m_status.m_grid = std::make_unique<Grid>(Grid(m,m));
		ds->m_status.m_grid->getCellAt(m/2,m/2) = 1000.0;
	}
	, [](void* targetValue, void* userData)
	{
		uint32_t* m = reinterpret_cast<uint32_t*>(targetValue);
		auto ds = reinterpret_cast<DiffusionSimulator*>(userData);
		*m = uint32_t(ds->m_status.m_grid->getWidth());
	}, this, "min=1");
	TwAddVarCB(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, [](const void* targetValue, void* userData)
	{
		auto alpha = *reinterpret_cast<const float*>(targetValue);
		auto ds = reinterpret_cast<DiffusionSimulator*>(userData);
		ds->m_status.m_alpha = alpha;
	}
		, [](void* targetValue, void* userData)
	{
		auto m = reinterpret_cast<float*>(targetValue);
		auto ds = reinterpret_cast<DiffusionSimulator*>(userData);
		*m = ds->m_status.m_alpha;
	}, this, "");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//

	switch (m_iTestCase)
	{
	case 0:
		{
			cout << "Reset Explicit solver!\n";
			auto& grid = this->m_status.m_grid;
			// Fill with zero
			grid->fill(0.0);
			grid->getCellAt(grid->getWidth() / 2, grid->getHeight() / 2) = 200.0;
		}
	break;
	case 1:
		{
			cout << "Implicit solver!\n";
			auto& grid = this->m_status.m_grid;
			// Fill with zero
			grid->fill(0.0);
			grid->getCellAt(grid->getWidth() / 2, grid->getHeight() / 2) = 200.0;
		}
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}

}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(SimulationStatus& simulation, float deltaTime) {//add your own parameters
	const size_t size[2] = { simulation.m_grid->getWidth(), simulation.m_grid->getHeight() };
	auto& newT = std::make_unique<Grid>(Grid(size[0],size[1]));
	
	auto gridPtr = simulation.m_grid.get();
	//make sure that the temperature in boundary cells stays zero
	const auto& T = [gridPtr](int x, int y) { 
		if ((x < 0 || y < 0) || (x >= gridPtr->getWidth() || y >= gridPtr->getHeight()))
			return 0.0; 
		return gridPtr->getCellAt(x, y); 
	};
	//const auto& Told = [&](int x, int y) { if ((x < 0 || y < 0) || (x >= simulation.m_grid->getWidth() || y >= simulation.m_grid->getHeight())) return 0.0; else simulation.m_previous->getCellAt(x, y); };
	const auto alpha = simulation.m_alpha;
	const auto stepH = 1;
	const auto stepV = 1;

	//const auto stepH = pow(1.0 / gridPtr->getWidth(), 2);
	//const auto stepV = pow(1.0 / gridPtr->getHeight(), 2);

	for (size_t y = 0; y < size[1]; y++)
	{
		for (size_t x = 0; x < size[0]; x++)
		{
			// compute dTdt: see slides to use forward method for time derivative and central method for dT2dx2 and dT2dy2
			auto dTdt = alpha * ((T(x + 1, y) - 2 * T(x, y) + T(x - 1, y))/stepH + (T(x, y + 1) - 2 * T(x, y) + T(x, y - 1))/stepV) * deltaTime + T(x, y);;
			// Integrate using Euler
			//auto newTcell = dTdt*deltaTime+T(x,y);
			// Store result
			newT->getCellAt(x, y) = dTdt;

			//std::cout << "Cell diff [" << x << ", " << y << "]: " << (newT->getCellAt(x, y) - T(x, y)) << std::endl;
		}
	}
	simulation.m_previous = std::move(simulation.m_grid);
	simulation.m_grid = std::move(newT);
	std::cout << "Delta time " << deltaTime << std::endl;
	return simulation.m_grid.get();
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	/*for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}*/
}

void fillT(std::vector<Real>& b,Grid& g) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	const size_t size[2] = { g.getWidth(),g.getHeight() };
	const auto width = size[0];
	const auto height = size[1];
	for (size_t i = 0; i < size[0] * size[1]; i++)
	{
		size_t x = i % width;
		size_t y = i / width;
		g.getCellAt(x, y) = b[i];
	}

}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}

void setupImplicitEuler2D(Grid& g, SparseMatrix<Real>& A, std::vector<Real>& b, float deltaTime, float deltaPos,float alpha)
{
	const size_t size[2] = { g.getWidth(),g.getHeight() };
	const auto width = size[0];
	const auto height = size[1];

	const auto& grid = &g;

	const auto calculateIndex = [width, height](signed int x, signed int y) {
		if (x < 0 || y < 0)
			return -1;
		if (x >= width || y >= height)
			return -1;
		return static_cast<signed int>(width * y + x);
	};

	//const auto deltaTime = 0.1;
	const auto deltaPosition = pow(1,2);
	//const auto alpha = 0.1;
	const auto F = -alpha * deltaTime;

	static std::vector<std::vector<char>> indices = { { -1,-1 },{ -1,-0 },{ 0,-1 },{ 0,1 },{ 1,0 },{ 1,1 } };
	// For each Ti,j, we set up a single row in A matrix, containing values for parameters Ti+1,j...etc at time N
	// See 'http://hplgit.github.io/num-methods-for-PDEs/doc/pub/diffu/sphinx/._main_diffu001.html' to understand what is Ax=b
	for (size_t y = 0; y < height; y++) {

		for (size_t x = 0; x < width; x++) {
			auto index = calculateIndex(x, y);
			// for each neighbour
			for (auto& neighbour : indices)
			{
				auto neighbourIndex = calculateIndex(x + neighbour[0], y + neighbour[1]);
				if (neighbourIndex == -1)
					continue;
				A.set_element(index, neighbourIndex, F);
			}
			// set our index
			A.set_element(index, index, deltaPosition - 4 * F);
			// set our b 
			b.at(index) = grid->getCellAt(x, y)*deltaPosition;
		}
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit(float deltaTime) {//add your own parameters
	// solve A T = b
	// to be implemented
	const size_t size[2] = { m_status.m_grid->getWidth(),m_status.m_grid->getHeight() };
	const auto width = size[0];
	const auto height = size[1];

	const int N = width*height;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);
	//A->zero();

	//setupA(*A, 0.1);
	//setupB(*b);
	setupImplicitEuler2D(*m_status.m_grid,*A, *b,deltaTime,1.0,m_status.m_alpha);

	/*std::cout << "Matrix " << std::endl;
	for (size_t y = 0; y < N; y++)
	{
		for (size_t x = 0; x < N; x++)
		{
			std::cout << " " << setw(4) << A->operator()(x, y);
		}
		std::cout << std::endl;
	}
	std::cout << setw(1);

	std::cout << "B vec: " << std::endl;
	for (size_t y = 0; y < N; y++)
	{
		std::cout << " " << (*b)[y];
	}	
	std::cout << std::endl;


	std::cout << "b: " << b << std::endl;
	*/
	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x,*m_status.m_grid);//copy x to T
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(this->m_status,timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization

	for (size_t x = 0; x < T->getWidth(); x++)
	{
		for (size_t y = 0; y < T->getHeight(); y++)
		{
			const double scX = 1.0 / T->getWidth();
			const double scY = 1.0 / T->getHeight();
			auto temp = T->getCellAt(x, y);
			auto const maxTemp = 100.0;
			// Interpolation parameter t
			auto t = temp / maxTemp;

			const double scZ = min(scX, scY);
			auto pos = Vec3(x*scX,t*scZ, y*scY);
			auto size = Vec3(scX, scZ, scY);

			// Interpolate between white and red color
			auto color = Vec3(1.0, 0.0, 0.0)*t + (1.0 - t)*Vec3(1.0);
			// Draw
			this->DUC->setUpLighting(Vec3(0.0), Vec3(1.0), 0.1, color);
			this->DUC->drawSphere(pos, size/2.0);
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
