#pragma once
#include "MeshEdit/MeshEditor.h"
#include "MeshEdit/Parameterization/ParaEnum.h"
#include <Eigen/Sparse>
#include <iostream>
using namespace std;
using namespace Eigen;

class Parameterization : public MeshEditor
{
protected:
	

public:
	Parameterization(shared_ptr<Mesh> mesh_ptr);
	Parameterization(Parameterization* param);

	~Parameterization() { cout << "####" << endl; }

	virtual bool Run() { cout << "pppp" << endl;  return true; }
	virtual bool Init() { return true; }
	virtual bool Iterate() { return true; }
	virtual bool IterateTillDone() { return true; }

	void setViewMode(ViewMode mode) { 
		view_mode = mode; 
		if (init_status) UpdateMesh();
	}
	void setInitMethod(InitMethod method) {
		init_method = method;
	}
	ViewMode getViewMode() { return view_mode; }

private:
	void init_mesh_info();

protected:
	virtual void UpdateMesh() { }
	void build_cot_lap();
	void build_uni_lap();
	Vector3d Barycenter(MatrixXd map);

protected:
	ViewMode	view_mode;
	InitMethod	init_method;
	size_t		iter_count;
	bool		is_iter_method;
	bool		init_status;
	bool		finish_status;
	bool		cot_lap_avail;
	bool		uni_lap_avail;

	SparseMatrix<double, RowMajor>	CotL;	// Cotangent Laplacian Matrix
	SparseMatrix<double, RowMajor>	UniL;	// Uniform Laplacian Matrix
};

