#pragma once
// SLIM
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
	class SLIM : public NoFlipPipeline {
	public:
		SLIM(Ptr<TriMesh> triMesh) : NoFlipPipeline(triMesh) { }
		SLIM(Ptr<Parameterization> embd) : NoFlipPipeline(embd) { }
		SLIM(Ptr<Preprocessing> preproc) : NoFlipPipeline(preproc) { }
	public:
		static const Ptr<SLIM> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<SLIM>(triMesh);
		}
		static const Ptr<SLIM> New(Ptr<Parameterization> embd) {
			return Ubpa::New<SLIM>(embd);
		}
		static const Ptr<SLIM> New(Ptr<Preprocessing> preproc) {
			return Ubpa::New<SLIM>(preproc);
		}
	public:
		bool Iterate();

	};
}

