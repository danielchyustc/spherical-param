#pragma once
// CM
#include <Engine/MeshEdit/Parameterization/PlanarEmbedding/NoFlipPipeline.h>
#include <Engine/MeshEdit/Parameterization/PardisoSolver.h>
#include <Engine/MeshEdit/Preprocessing.h>
#include <UHEMesh/HEMesh.h>
#include <UGM/UGM>
#include <Eigen/Sparse>
#include <Eigen/Dense>

using namespace Eigen;

namespace Ubpa {
	class TriMesh;
	class Preprocessing;
	enum ViewMode;
	// mesh boundary == 1
	class CM : public NoFlipPipeline {
	public:
		CM(Ptr<TriMesh> triMesh) : NoFlipPipeline(triMesh) { }
		CM(Ptr<Parameterization> embd) : NoFlipPipeline(embd) { }
		CM(Ptr<Preprocessing> preproc) : NoFlipPipeline(preproc) { }
	public:
		static const Ptr<CM> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<CM>(triMesh);
		}
		static const Ptr<CM> New(Ptr<Parameterization> embd) {
			return Ubpa::New<CM>(embd);
		}
		static const Ptr<CM> New(Ptr<Preprocessing> preproc) {
			return Ubpa::New<CM>(preproc);
		}
	public:
		bool Iterate();

	};
}
