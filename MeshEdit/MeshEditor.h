#pragma once
#include "MeshDefinition.h"
#include <Eigen/Dense>
#include <memory>
#include <iostream>

using namespace std;
using namespace Eigen;

class MeshEditor : public enable_shared_from_this<MeshEditor>
{
protected:
	shared_ptr<Mesh> mesh, orig_mesh;
	size_t nV, nE, nHE, nF;

public:
	MeshEditor() { }
	~MeshEditor() { }
	MeshEditor(shared_ptr<Mesh> mesh_ptr);
	static const shared_ptr<MeshEditor> New(shared_ptr<Mesh> mesh_ptr) {
		return make_shared<MeshEditor>(MeshEditor(mesh_ptr));
	}

	virtual bool Run();
	bool Reset();

private:
	void init_mesh_info();

protected:
	vector<vector<size_t>>	tri_vert;	// triangle - vertex indices
		// tri_vert[t][k] is the index of the kth (0 <= k < 3) vertex of the tth (0 <= t < nT) triangle
	vector<vector<size_t>>	tri_graph;	// triangle - triangle connectivity
	// tri_graph[t][k] is the index of the kth (0 <= k < 3) neighbor triangle of the tth (0 <= t < nT) triangle
	MatrixXd	input_mesh_pos;
	MatrixXd	para_map;
	VectorXd	area_weight;
	VectorXd	voronoi_weight;

};

