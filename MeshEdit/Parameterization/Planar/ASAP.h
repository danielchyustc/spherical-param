#pragma once

#include <Engine/MeshEdit/Parameterization/PlanarEmbedding/PlanarEmbedding.h>
#include <Engine/MeshEdit/Preprocessing.h>
#include <UHEMesh/HEMesh.h>
#include <UI/Attribute.h>
#include <UGM/UGM>
#include <Eigen/Sparse>
#include <Eigen/Dense>

using namespace Eigen;

namespace Ubpa {
	class TriMesh;
	class Preprocessing;
	enum ViewMode;
	// mesh boundary == 1
	class ASAP : public PlanarEmbedding {
	public:
		ASAP(Ptr<TriMesh> triMesh) : PlanarEmbedding(triMesh) { }
		ASAP(Ptr<Parameterization> embd) : PlanarEmbedding(embd) { }
		ASAP(Ptr<Preprocessing> preproc) : PlanarEmbedding(preproc) { }
	public:
		static const Ptr<ASAP> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<ASAP>(triMesh);
		}
		static const Ptr<ASAP> New(Ptr<Parameterization> embd) {
			return Ubpa::New<ASAP>(embd);
		}
		static const Ptr<ASAP> New(Ptr<Preprocessing> preproc) {
			return Ubpa::New<ASAP>(preproc);
		}
	public:
		bool Run();

	protected:
		void InitFlatTri();
		void UpdateParaMap();
		void UpdateTriMesh();

	protected:
		Matrix<double, 3, 2>				*flat_tri_;
	};
}

