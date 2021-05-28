#include "MeshEdit/MinSurf.h"
#include <Eigen/Sparse>
#include <iostream>

using namespace Eigen;
using namespace std;

bool MinSurf::Run()
{
	if (!MeshTools::HasBoundary(*mesh))
	{
		cout << "Error::The mesh does not have any boundaries!" << endl;
		return false;
	}

	SparseMatrix<double> Laplacian(nV, nV);
	MatrixXd RHS(nV, 3);
	OpenMesh::Vec3d point;
	size_t i, j;
	double weight = 1;
	RHS.setZero();
	for (auto vh : mesh->vertices())
	{
		i = vh.idx();
		if (mesh->is_boundary(vh))
		{
			Laplacian.insert(i, i) = 1;
			point = mesh->point(vh);
			RHS.row(i) << point[0], point[1], point[2];
		}
		else
		{
			for (auto heh : mesh->voh_range(vh))
			{
				j = mesh->to_vertex_handle(heh).idx();
				weight = MeshTools::CotanWeight(*mesh, heh);
				Laplacian.coeffRef(i, i) += weight;
				Laplacian.insert(i, j) = -weight;
			}
		}
	}

	Laplacian.makeCompressed();
	SparseLU<SparseMatrix<double>> solver;
	solver.compute(Laplacian);
	MatrixXd SOL = solver.solve(RHS);

	for (auto vh : mesh->vertices())
	{
		i = vh.idx();
		mesh->set_point(vh, { SOL(i, 0), SOL(i, 1), SOL(i, 2) });
	}
	return true;
}