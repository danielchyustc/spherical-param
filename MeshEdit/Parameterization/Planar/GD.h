#pragma once
// Gradient Descent
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
	class Paramaterize;
	enum ViewMode;
	// mesh boundary == 1
	class GD : public NoFlipPipeline {
	public:
		GD(Ptr<TriMesh> triMesh) : NoFlipPipeline(triMesh) { }
		GD(Ptr<Parameterization> embd) : NoFlipPipeline(embd) { }
		GD(Ptr<Preprocessing> preproc) : NoFlipPipeline(preproc) { }
	public:
		static const Ptr<GD> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<GD>(triMesh);
		}
		static const Ptr<GD> New(Ptr<Parameterization> embd) {
			return Ubpa::New<GD>(embd);
		}
		static const Ptr<GD> New(Ptr<Preprocessing> preproc) {
			return Ubpa::New<GD>(preproc);
		}

}

