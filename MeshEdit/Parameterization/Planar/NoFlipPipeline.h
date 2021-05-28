#pragma once
// No Flip Pipeline
#include <Engine/MeshEdit/Parameterization/PlanarEmbedding/PlanarEmbedding.h>
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
	class NoFlipPipeline : public PlanarEmbedding {
	public:
		NoFlipPipeline(Ptr<TriMesh> triMesh) : PlanarEmbedding(triMesh) { }
		NoFlipPipeline(Ptr<Parameterization> embd) : PlanarEmbedding(embd) { }
		NoFlipPipeline(Ptr<Preprocessing> preproc) : PlanarEmbedding(preproc) { }
	public:
		static const Ptr<NoFlipPipeline> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<NoFlipPipeline>(triMesh);
		}
		static const Ptr<NoFlipPipeline> New(Ptr<Parameterization> embd) {
			return Ubpa::New<NoFlipPipeline>(embd);
		}
		static const Ptr<NoFlipPipeline> New(Ptr<Preprocessing> preproc) {
			return Ubpa::New<NoFlipPipeline>(preproc);
		}
	public:
		bool Run();
		bool Iterate();

	protected:
		void InitPara();
		void UpdateTriMesh();
		void Tutte();

		void source_local_coordinate_inverse();
		void update_reference();
		void sym_dir_energy();
		void sym_dir_energy(VectorXd pos, double& e);
		void compute_distort_weight();
		void compute_gradient();
		void compute_solver_AQP();
		void compute_solver_AKVF();
		void compute_solver_SLIM();
		void compute_solver_CM();
		void solve_velocity();
		void solve_step();
		void solve_distortion(double& sig1, double& sig2);
		void max_step();
		void backtracking_line_search();
		void filter(VectorXd x);

		MatrixXd source_lci, ref_lci, lci;		//local coordinate inverse
		double energy;
		VectorXd gradient;

		VectorXd distort_weight;

		VectorXd position, velocity;
		double step;
		SparseLU<SparseMatrix<double>>	solver;

	};
}

