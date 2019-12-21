#include "TetGenImporter.h"
#include <iostream>
#include <fstream>

/**
 * Returns the last Win32 error, in string format.Returns an empty string if there is no error.
 */
std::string GetLastErrorAsString()
{
	//Get the error message, if any.
	DWORD errorMessageID = ::GetLastError();
	if (errorMessageID == 0)
		return std::string(); //No error message has been recorded

	LPSTR messageBuffer = nullptr;
	size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

	std::string message(messageBuffer, size);

	//Free the buffer.
	LocalFree(messageBuffer);

	return message;
}

char const * loadStringResource(LPCWSTR resId, LPCWSTR resType)
{
	HRSRC resourcePtr = FindResource(NULL, resId, resType);	
	if(!resourcePtr)
	{
		std::cerr << GetLastErrorAsString() << std::endl;
		std::cin.get();
		exit(GetLastError());
	}
	HGLOBAL resourceData = LoadResource(0, resourcePtr);
	LPVOID lockedResource = LockResource(resourceData);
	return static_cast<char const*>(lockedResource);
}

import::TetGenImporter::TetGenImporter()
	: m_vertices()
	, m_edges()
	, m_faces()
{
	
}

import::TetGenImporter::TetGenImporter(LPCWSTR Name)
	:	m_objectName(Name)
{
}

void import::TetGenImporter::GetPrimitives(std::vector<Particle>& outParticles, std::vector<Spring>& outSprings)
{
	ReadVertices();
	ReadTets();

	outParticles.clear();
	for(std::size_t i = 0; i < m_vertices.size(); ++i)
	{		
		outParticles.emplace_back(Particle(m_vertices[i]));
	}

	for (std::size_t i = 0; i < m_edges.size(); ++i)
	{
		Edge& edge = m_edges[i];
		Vec3& a = m_vertices[edge.vertexA];
		Vec3& b = m_vertices[edge.vertexB];
		outSprings.emplace_back(Spring(edge.vertexA, edge.vertexB, b - a, 1));
	}
}

void import::TetGenImporter::ReadVertices()
{
	m_vertices.clear();
	std::wstring node = L"NODE";	
	char const* nodeString = loadStringResource((m_objectName+L"_"+node).c_str(),node.c_str());
	std::stringstream nodesStream = std::stringstream(nodeString);

	std::size_t numVerts, dimensions, unknown1, unknown2;
	nodesStream >> numVerts >> dimensions >> unknown1 >> unknown2;
	assert(dimensions == 3);
	for(std::size_t i = 0; i < numVerts; ++i)
	{
		std::size_t index{};
		Vec3 vertex{};

		nodesStream >> index >> vertex.x >> vertex.y >> vertex.z;
		assert(index == i);
		vertex *= 0.2;
		m_vertices.push_back(vertex);
	}
}

void import::TetGenImporter::ReadEdges()
{
	m_edges.clear();
	std::wstring edge = L"EDGE";
	char const* nodeString = loadStringResource((m_objectName + L"_" + edge).c_str(), edge.c_str());
	std::stringstream edgeStream{ nodeString };
	std::size_t numEdges;
	int maxDepth;
	edgeStream >> numEdges >> maxDepth;
	for (std::size_t i = 0; i < numEdges; ++i)
	{
		std::size_t index{};
		Edge edge{};
		int depth{};

		edgeStream >> index >> edge.vertexA >> edge.vertexB >> depth;
		assert(index == i);
		m_edges.push_back(edge);
	}
}

void import::TetGenImporter::ReadTets()
{
	m_edges.clear();

	std::wstring ele = L"ELE";
	char const* eleString = loadStringResource((m_objectName + L"_" + ele).c_str(), ele.c_str());
	std::stringstream eleStream = std::stringstream(eleString);

	std::size_t numTets, dimensions, unknown1;
	eleStream >> numTets >> dimensions >> unknown1;
	assert(dimensions == 4);
	for (std::size_t i = 0; i < numTets; ++i)
	{
		std::vector<std::pair<char, char>> connectionList = 
		{
			{0,1}, {0,2}, {0,3},
			{1,2}, {1,3},
			{2,3}
		};

		std::size_t vertices[4];
		std::size_t index;
		eleStream >> index >> vertices[0] >> vertices[1] >> vertices[2] >> vertices[3];

		for(auto edgeIndex : connectionList)
		{
			Edge edge;
			edge.vertexA = vertices[edgeIndex.first];
			edge.vertexB = vertices[edgeIndex.second];
			m_edges.push_back(edge);
		}
	}
}
