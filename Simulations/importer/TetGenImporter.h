#pragma once
#include "SpringsImporter.h"
#include <Windows.h>
#include "../resource.h"

namespace import
{
	class TetGenImporter : public SpringsImporter
	{
	public:
		TetGenImporter();
		TetGenImporter(LPCWSTR Name);
		
		void GetPrimitives(std::vector<Particle>& outParticles, std::vector<Spring>& outSprings) override;

	protected:
		std::vector<Vec3> m_vertices;
		std::vector<Edge> m_edges;
		std::vector<Face> m_faces;

		std::wstring m_objectName;
		
		void ReadVertices();
		void ReadEdges();
		void ReadTets();
		void ReadFaces(){};
	};

}
