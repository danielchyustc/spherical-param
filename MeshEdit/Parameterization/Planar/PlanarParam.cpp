#include "MeshEdit/Parameterization/Planar/PlanarParam.h"
#include <Eigen/Sparse>

using namespace std;
using namespace Eigen;

PlanarParam::PlanarParam(shared_ptr<Mesh> mesh_ptr) :
	Parameterization(mesh_ptr)
{
	view_mode = kPlane;
}

PlanarParam::PlanarParam(Parameterization* param) :
	Parameterization(param)
{
	view_mode = kPlane;
}

bool PlanarParam::Run() {
	para_map.resize(2, nV);
	init_status = true;
	UpdateMesh();
	
	return true;
}

void PlanarParam::UpdateMesh()
{
	Vector3d point;
	if (view_mode == kMesh) *mesh = *orig_mesh;
	for (size_t i = 0; i < nV; i++)
	{
		auto vi = mesh->vertex_handle(i);
		point = para_map.col(i);
		if(view_mode == kPlane)	mesh->set_point(vi, { point[0], point[1], 0 });
		mesh->set_texcoord2D(vi, { point[0], point[1] });
	}
}