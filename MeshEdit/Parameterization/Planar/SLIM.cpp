#include <Engine/MeshEdit/Parameterization/PlanarEmbedding/SLIM.h>
#include <Engine/Primitive/TriMesh.h>
#include <gsl/gsl_poly.h>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SVD> 

using namespace Ubpa;

using namespace std;
using namespace Eigen;

bool SLIM::Iterate()
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

	for (size_t i = 0; i < 10 && !finish_status_; i++)
	{
		compute_gradient();
		compute_solver_SLIM();
		solve_velocity();
		solve_step();
		position += velocity * step;
		cout << ++iter_count_ << "th iteration done!" << endl;

		double energy_pre = energy;
		sym_dir_energy();
		if (fabs(energy - energy_pre) / energy < 1e-6)
			finish_status_ = true;

		update_reference();
		lci = ref_lci;
	}
	
	for (size_t i = 0; i < nV; i++)
	{
		para_map_(i, 0) = position(i);
		para_map_(i, 1) = position(i + nV);
	}
	UpdateTriMesh();
	return true;
}
