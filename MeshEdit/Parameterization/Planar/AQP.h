#pragma once
// Cotan Weight Gradient Descent
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
	class AQP : public NoFlipPipeline {
	public:
		AQP(Ptr<TriMesh> triMesh) : NoFlipPipeline(triMesh) { }
		AQP(Ptr<Parameterization> embd) : NoFlipPipeline(embd) { }
		AQP(Ptr<Preprocessing> preproc) : NoFlipPipeline(preproc) { }
	public:
		static const Ptr<AQP> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<AQP>(triMesh);
		}
		static const Ptr<AQP> New(Ptr<Parameterization> embd) {
			return Ubpa::New<AQP>(embd);
		}
		static const Ptr<AQP> New(Ptr<Preprocessing> preproc) {
			return Ubpa::New<AQP>(preproc);
		}
	public:
		bool Run();
		bool Iterate();

	protected:
		void InitPara();
		void solve_velocity();

	};
}

