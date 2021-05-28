#include "MeshEdit/Parameterization/Spherical/GaussMap.h"
#include <iostream>

GaussMap::GaussMap(shared_ptr<Mesh> mesh_ptr) : 
	SphericalParam(mesh_ptr) 
{
	is_iter_method = false;
}

GaussMap::GaussMap(Parameterization* param) :
	SphericalParam(param)
{
	is_iter_method = false;
}

bool GaussMap::Run()
{
	if (MeshTools::HasBoundary(*mesh))
	{
		cout << "Error: Only for closed meshes!" << endl;
	}
	cout << "g001" << endl;
	InitParaMap();
	cout << "g002" << endl;
	UpdateMesh();
	cout << "g003" << endl;
	return true;
}

void GaussMap::InitParaMap()
{
	OpenMesh::Vec3d point;
	para_map.resize(3, nV);
	for (auto vh : mesh->vertices())
	{
		point = mesh->normal(vh);
		para_map.col(vh.idx()) << point[0], point[1], point[2];
	}
	init_status = true;
	finish_status = true;
	cout << "0000" << endl;
}
