#include "MeshEdit/Parameterization/Parameterization.h"
#include <iostream>

Parameterization::Parameterization(shared_ptr<Mesh> mesh_ptr) : 
	MeshEditor(mesh_ptr), 
	is_iter_method(false) 
{ 
	init_mesh_info();
}

Parameterization::Parameterization(Parameterization* param) :
	MeshEditor(param->orig_mesh)
{
	if (!param->init_status) init_mesh_info();
	else
	{
		view_mode = param->view_mode;
		init_method = param->init_method;
		init_status = param->finish_status;
		finish_status = false;
		cot_lap_avail = param->cot_lap_avail;
		uni_lap_avail = param->uni_lap_avail;
		iter_count = 0;
		if (param->finish_status) para_map = param->para_map;
		if (param->cot_lap_avail) CotL = param->CotL;
		if (param->uni_lap_avail) UniL = param->UniL;
	}
}

void Parameterization::init_mesh_info()
{
	if (MeshTools::HasBoundary(*mesh)) view_mode = kPlane;
	else view_mode = kSphere;
	init_method = kMeshVoronoi;
	init_status = false;
	finish_status = false;
	cot_lap_avail = false;
	uni_lap_avail = false;
	iter_count = 0;
}

void Parameterization::build_cot_lap()
{
	cout << "Building cotangent laplacian matrix..." << endl;
	CotL.resize(nV, nV);
	size_t i, j;
	double weight;
	for (auto vh : mesh->vertices())
	{
		i = vh.idx();
		for (auto heh : mesh->voh_range(vh))
		{
			j = mesh->to_vertex_handle(heh).idx();
			weight = MeshTools::CotanWeight(*mesh, heh);
			CotL.coeffRef(i, i) += weight;
			CotL.insert(i, j) = -weight;
		}
		if ((i + 1) % 10000 == 0)
		{
			cout << i + 1 << " rows built." << endl;
		}
	}
	CotL.makeCompressed();
	cot_lap_avail = true;
	cout << "Cotangent laplacian matrix built." << endl;
}

void Parameterization::build_uni_lap()
{
	cout << "Building uniform laplacian matrix..." << endl;
	UniL.resize(nV, nV);
	size_t i, j;
	for (auto vh : mesh->vertices())
	{
		i = vh.idx();
		for (auto heh : mesh->voh_range(vh))
		{
			j = mesh->to_vertex_handle(heh).idx();
			UniL.coeffRef(i, i) += 1;
			UniL.insert(i, j) = -1;
		}
		if ((i + 1) % 10000 == 0)
		{
			cout << i + 1 << " rows built." << endl;
		}
	}
	UniL.makeCompressed();
	uni_lap_avail = true;
	cout << "Uniform laplacian matrix built." << endl;
}

Vector3d Parameterization::Barycenter(MatrixXd map)
{
	Vector3d center = { 0, 0, 0 };
	for (size_t i = 0; i < nV; i++) center += voronoi_weight[i] * map.col(i);
	return center;
}