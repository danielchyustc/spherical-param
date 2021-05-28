#pragma once
#include "MeshEdit/MeshEditor.h"

class MinSurf : public MeshEditor
{
public:
	MinSurf(shared_ptr<Mesh> mesh_ptr) : MeshEditor(mesh_ptr) { }
	shared_ptr<MinSurf> New(shared_ptr<Mesh> mesh_ptr) {
		return make_shared<MinSurf>(MinSurf(mesh_ptr));
	}

	bool Run() override;

};

