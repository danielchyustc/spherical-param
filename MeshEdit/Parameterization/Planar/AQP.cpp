#include <Engine/MeshEdit/Parameterization/PlanarEmbedding/AQP.h>
#include <Engine/Primitive/TriMesh.h>
#include <gsl/gsl_poly.h>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SVD> 

using namespace Ubpa;

using namespace std;
using namespace Eigen;

bool AQP::Run() {
	if (heMesh->IsEmpty() || !triMesh)
	{
		printf("ERROR::Minsurf::Run\n"
			"\t""heMesh->IsEmpty() || !triMesh\n");
		return false;
	}

	InitPara();
	iter_count_ = 0;
	init_status_ = true;
	finish_status_ = false;

	UpdateTriMesh();
	return true;
}

void AQP::InitPara()
{
	position.resize(2 * nV);
	velocity.resize(2 * nV);
	para_map_.resize(nV, 2);
	step = 0;
	velocity.setZero();
	source_local_coordinate_inverse();
	cout << "4";
	Tutte();
	cout << "5";
	gradient.resize(2 * nV);
	ref_lci.resize(nT, 4);
	lci.resize(nT, 4);
	update_reference();
	cout << "6";
	lci = ref_lci;
	sym_dir_energy();
	cout << "7";
	compute_solver_AQP();
	cout << "8" << endl;
}

bool AQP::Iterate()
{
	if (iter_count_ >= 10000)
	{
		cout << "Have reached maximal iteration number!" << endl;
		return true;
	}
	if (finish_status_)
	{
		cout << "Already finished!" << endl;
		return true;
	}

	for (size_t i = 0; i < 10; i++)
	{
		compute_gradient();
		solve_velocity();
		solve_step();
		position += velocity * step;

		double energy_pre = energy;
		sym_dir_energy();
		if (fabs(energy - energy_pre) / energy < 1e-6)
			finish_status_ = true;

		update_reference();
		lci = ref_lci;
	}

	iter_count_++;

	cout << iter_count_ << "th iteration done!" << endl;
	for (size_t i = 0; i < nV; i++)
	{
		para_map_(i, 0) = position(i);
		para_map_(i, 1) = position(i + nV);
	}
	UpdateTriMesh();
	return true;
}

void AQP::solve_velocity()
{
	MatrixXd RHS(nV, 2), sol(nV, 2);
	for (size_t i = 0; i < nV; i++)
	{
		RHS(i, 0) = -gradient[i];
		RHS(i, 1) = -gradient[i + nV];
	}
	sol = solver.solve(RHS);
	for (size_t i = 0; i < nV; i++)
	{
		velocity[i] = sol(i, 0);
		velocity[i + nV] = sol(i, 1);
	}
	filter(velocity);
	cout << velocity.norm() << "\t";
}
