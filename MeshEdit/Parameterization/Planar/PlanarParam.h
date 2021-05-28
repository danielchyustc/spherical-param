#pragma once

#include <Eigen/Sparse>
#include "MeshEdit/Parameterization/Parameterization.h"

using namespace Eigen;

// mesh boundary > 0
class PlanarParam : public Parameterization {
public:
	PlanarParam(shared_ptr<Mesh> mesh_ptr);
	PlanarParam(Parameterization* param);
public:
	virtual bool Run() override;
	virtual bool Iterate() { return true; }
	virtual bool IterateTillDone() { return true; }
	void SetViewMode(ViewMode mode) {
		view_mode = mode;
		if (init_status) UpdateMesh();
	}

protected:
	virtual void UpdateParaMap() { }
	void UpdateMesh() override;

};
