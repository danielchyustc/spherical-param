#pragma once
// Composed Hessian Metric
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
	class AKVF : public NoFlipPipeline {
	public:
		AKVF(Ptr<TriMesh> triMesh) : NoFlipPipeline(triMesh) { }
		AKVF(Ptr<Parameterization> embd) : NoFlipPipeline(embd) { }
		AKVF(Ptr<Preprocessing> preproc) : NoFlipPipeline(preproc) { }
	public:
		static const Ptr<AKVF> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<AKVF>(triMesh);
		}
		static const Ptr<AKVF> New(Ptr<Parameterization> embd) {
			return Ubpa::New<AKVF>(embd);
		}
		static const Ptr<AKVF> New(Ptr<Preprocessing> preproc) {
			return Ubpa::New<AKVF>(preproc);
		}

	public:
		bool Iterate();

	};
}

