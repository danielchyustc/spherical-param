#include "MeshEdit/MeshEditor.h"

MeshEditor::MeshEditor(shared_ptr<Mesh> mesh_ptr) :
	mesh(shared_ptr<Mesh>(mesh_ptr)),
	orig_mesh(make_shared<Mesh>(*mesh_ptr))
{ 
	init_mesh_info();
}

void MeshEditor::init_mesh_info()
{
	nV = mesh->n_vertices();
	nE = mesh->n_edges();
	nHE = mesh->n_halfedges();
	nF = mesh->n_faces();

	tri_vert.resize(nF);
	tri_graph.resize(nF);
	area_weight.resize(nF);
	voronoi_weight.resize(nV);
	voronoi_weight.setZero();
	size_t fi;
	double area_f, total_area = 0;
	OpenMesh::VertexHandle vh0, vh1, vh2;
	OpenMesh::HalfedgeHandle heh;
	for (auto fh : mesh->faces())
	{
		fi = fh.idx();
		heh = fh.halfedge();
		tri_vert[fi] = { (size_t)mesh->from_vertex_handle(heh).idx(), (size_t)mesh->to_vertex_handle(heh).idx(),
			(size_t)mesh->to_vertex_handle(mesh->next_halfedge_handle(heh)).idx() };
		tri_graph[fi] = { (size_t)mesh->opposite_face_handle(heh).idx(),
			(size_t)mesh->opposite_face_handle(mesh->next_halfedge_handle(heh)).idx(),
			(size_t)mesh->opposite_face_handle(mesh->prev_halfedge_handle(heh)).idx() };

		area_f = mesh->calc_face_area(fh);
		area_weight[fi] = area_f;
		total_area += area_f;
		for (size_t k = 0; k < 3; k++) voronoi_weight[tri_vert[fi][k]] += area_f;
	}
	area_weight /= total_area;
	voronoi_weight /= (3 * total_area);

	input_mesh_pos.resize(3, nV);
	double area_coeff = sqrt(1.0 / total_area);
	OpenMesh::Vec3d point;
	for (auto vh : mesh->vertices())
	{
		point = mesh->point(vh);
		input_mesh_pos.col(vh.idx()) = Vector3d(point[0], point[1], point[2]) * area_coeff;
	}
}

bool MeshEditor::Run()
{
	for (auto& vh : mesh->vertices())
		mesh->set_point(vh, mesh->point(vh) * 2);
	return true;
}

bool MeshEditor::Reset()
{
	*mesh = *orig_mesh;
	return true;
}
