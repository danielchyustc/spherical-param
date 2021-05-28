#include "MeshEdit/Parameterization/Planar/Progressive.h"

Progressive::Progressive(shared_ptr<Mesh> mesh_ptr) :
	PlanarParam(mesh_ptr)
{

}

Progressive::Progressive(Parameterization* param) :
	PlanarParam(param)
{

}

bool Progressive::Run()
{
	cout << "r001" << endl;
	init();
	cout << "r002" << endl;
	BPE();
	cout << "r003" << endl;
	UpdateMesh();
	cout << "r004" << endl;
	return true;
}

bool Progressive::Iterate()
{
	return true;
}

bool Progressive::IterateTillDone()
{
	return true;
}

void Progressive::init()
{
	cout << "i001" << endl;
	area.resize(nF);
	para_map.resize(2, nV);
	position_of_mesh.resize(2 * nV);
	negative_grad_norm.resize(2 * nV);
	cout << "i002" << endl;
	mesh->request_face_normals();
	mesh->update_normals();
	pardiso = NULL;
	convgence_con_rate = 1e-6;
	MAX_ITER_NUM = 5000;
	bound_distortion_K = 250;
	cout << "i003" << endl;
	double area_sum = 0.0;
	for (auto f_h : mesh->faces())
	{
		area_sum += mesh->calc_face_area(f_h);
	}
	originmesh_area_sqrt = sqrt(area_sum);
	cout << "i004" << endl;
	double area_same_factor = 1.0 / originmesh_area_sqrt;
	for (auto it1 : mesh->vertices())
	{
		mesh->set_point(it1, area_same_factor * mesh->point(it1));
	}
	cout << "i005" << endl;
	area.resize(nF);
	area_uniform.resize(nF, 1.0 / nF);
	area_src.resize(nF, 1.0 / nF);
	for (size_t i = 0; i < nF; i++) area[i] = area_weight[i];
	cout << "i006" << endl;
	Pre_calculate();
	cout << "i007" << endl;
	Tutte();
	cout << "i008" << endl;
	init_status = true;
}

void Progressive::Pre_calculate()
{
	cout << "p001" << endl;
	source_p00.resize(nF);
	source_p01.resize(nF);
	source_p10.resize(nF);
	source_p11.resize(nF);
	cout << "p002" << endl;
	for (int i = 0; i < nF; ++i)
	{
		double p00, p01, p10, p11;
		local_coordinate_inverse(i, p00, p01, p10, p11);

		source_p00[i] = p00;
		source_p01[i] = p01;
		source_p10[i] = p10;
		source_p11[i] = p11;
	}
	cout << "p003" << endl;
	update_p00 = source_p00;
	update_p01 = source_p01;
	update_p10 = source_p10;
	update_p11 = source_p11;
	cout << "p004" << endl;
	pardiso_i.clear(); pardiso_i.reserve(2 * nV + 1);
	pardiso_ia.clear(); pardiso_ia.reserve(2 * nV + 1);
	pardiso_ja.clear(); pardiso_ja.reserve(8 * nV);
	cout << "p005" << endl;
	typedef Triplet<int> T;
	std::vector<T> tripletlist;
	for (int i = 0; i < 2 * nV; i++)
	{
		pardiso_ia.push_back(pardiso_ja.size());
		if (i < nV)
		{
			auto vertex = mesh->vertex_handle(i);
			vector<int> row_id;

			row_id.push_back(i);
			row_id.push_back(i + nV);

			for (auto it : mesh->vv_range(vertex))
			{
				int id_neighbor = it.idx();
				row_id.push_back(id_neighbor);
				row_id.push_back(id_neighbor + nV);
			}
			std::sort(row_id.begin(), row_id.end(), less<int>());
			vector<int>::iterator iter = std::find(row_id.begin(), row_id.end(), i);

			int dd = 0;

			for (int k = std::distance(row_id.begin(), iter); k < row_id.size(); k++)
			{
				pardiso_ja.push_back(row_id[k]);
				pardiso_i.push_back(i);
				tripletlist.push_back(T(i, row_id[k], dd));
				++dd;
			}
		}
		else
		{
			auto vertex = mesh->vertex_handle(i - nV);

			vector<int> row_id;

			row_id.push_back(i);

			for (auto it : mesh->vv_range(vertex))
			{
				int id_neighbor = it.idx() + nV;
				row_id.push_back(id_neighbor);
			}
			std::sort(row_id.begin(), row_id.end(), less<int>());
			vector<int>::iterator iter = std::find(row_id.begin(), row_id.end(), i);

			int dd = 0;

			for (int k = std::distance(row_id.begin(), iter); k < row_id.size(); k++)
			{
				pardiso_ja.push_back(row_id[k]);
				pardiso_i.push_back(i);
				tripletlist.push_back(T(i, row_id[k], dd));
				++dd;
			}
		}
	}
	cout << "p006" << endl;
	SparseMatrix<int> find_id_in_rows;
	find_id_in_rows.resize(2 * nV, 2 * nV);
	find_id_in_rows.setFromTriplets(tripletlist.begin(), tripletlist.end());
	cout << "p007" << endl;
	pardiso_ia.push_back(pardiso_ja.size());

	id_h00.resize(nF); id_h01.resize(nF); id_h02.resize(nF); id_h03.resize(nF); id_h04.resize(nF); id_h05.resize(nF);
	id_h11.resize(nF); id_h12.resize(nF); id_h13.resize(nF); id_h14.resize(nF); id_h15.resize(nF);
	id_h22.resize(nF); id_h23.resize(nF); id_h24.resize(nF); id_h25.resize(nF);
	id_h33.resize(nF); id_h34.resize(nF); id_h35.resize(nF);
	id_h44.resize(nF); id_h45.resize(nF);
	id_h55.resize(nF);
	cout << "p008" << endl;
	for (int i = 0; i < nF; i++)
	{
		int f0 = tri_vert[i][0]; int f1 = tri_vert[i][1]; int f2 = tri_vert[i][2];
		int f3 = tri_vert[i][0] + nV; int f4 = tri_vert[i][1] + nV; int f5 = tri_vert[i][2] + nV;
		
		int min01 = min(f0, f1); int max01 = f0 + f1 - min01;
		int min02 = min(f0, f2); int max02 = f0 + f2 - min02;
		int min12 = min(f1, f2); int max12 = f1 + f2 - min12;
		
	//	cout << i << "\t" << f0 << "\t" << f1 << "\t" << f2 << "\t" << f3 << "\t" << f4 << "\t" << f5 << endl;

		id_h00[i] = pardiso_ia[f0]; id_h01[i] = pardiso_ia[min01] + find_id_in_rows.coeff(min01, max01); id_h02[i] = pardiso_ia[min02] + find_id_in_rows.coeff(min02, max02);
		id_h03[i] = pardiso_ia[f0] + find_id_in_rows.coeff(f0, f3); id_h04[i] = pardiso_ia[f0] + find_id_in_rows.coeff(f0, f4); id_h05[i] = pardiso_ia[f0] + find_id_in_rows.coeff(f0, f5);
		
		id_h11[i] = pardiso_ia[f1]; id_h12[i] = pardiso_ia[min12] + find_id_in_rows.coeff(min12, max12);
		id_h13[i] = pardiso_ia[f1] + find_id_in_rows.coeff(f1, f3); id_h14[i] = pardiso_ia[f1] + find_id_in_rows.coeff(f1, f4); id_h15[i] = pardiso_ia[f1] + find_id_in_rows.coeff(f1, f5);
		
		id_h22[i] = pardiso_ia[f2];
		id_h23[i] = pardiso_ia[f2] + find_id_in_rows.coeff(f2, f3); id_h24[i] = pardiso_ia[f2] + find_id_in_rows.coeff(f2, f4); id_h25[i] = pardiso_ia[f2] + find_id_in_rows.coeff(f2, f5);
		
		id_h33[i] = pardiso_ia[f3]; id_h34[i] = pardiso_ia[min01 + nV] + find_id_in_rows.coeff(min01 + nV, max01 + nV); id_h35[i] = pardiso_ia[min02 + nV] + find_id_in_rows.coeff(min02 + nV, max02 + nV);
		
		id_h44[i] = pardiso_ia[f4]; id_h45[i] = pardiso_ia[min12 + nV] + find_id_in_rows.coeff(min12 + nV, max12 + nV);
		
		id_h55[i] = pardiso_ia[f5];
		
	}
	cout << "p009" << endl;
}

void Progressive::local_coordinate_inverse(int i, double& p00, double& p01, double& p10, double& p11)
{
	int f0 = tri_vert[i][0];
	int f1 = tri_vert[i][1];
	int f2 = tri_vert[i][2];

	OpenMesh::Vec3d x_ = (mesh->point(mesh->vertex_handle(f1)) - mesh->point(mesh->vertex_handle(f0)));
	double x1_0 = x_.length();
	OpenMesh::Vec3d l_ = mesh->point(mesh->vertex_handle(f2)) - mesh->point(mesh->vertex_handle(f0));
	OpenMesh::Vec3d y_ = mesh->normal(mesh->face_handle(i)) % (1 / x1_0 * x_);
	double x2_0 = 1 / x1_0 * l_ | x_;
	double y2_0 = l_ | y_;

	p00 = 1 / x1_0;
	p01 = -x2_0 / (x1_0 * y2_0);
	p10 = 0;
	p11 = 1 / y2_0;
}

void Progressive::Tutte()
{
	int boundary_num = 0;
	auto it1 = mesh->halfedges_begin();
	while (!mesh->is_boundary(*it1))
		it1++;
	auto he_start = *it1;
	auto he_it = he_start;
	do
	{
		he_it = mesh->next_halfedge_handle(he_it);
		boundary_num++;
	} while (he_it != he_start);

	double delta_angle = 2 * M_PI / boundary_num;
	double area_1_factor = sqrt(1.0 / M_PI);

	position_of_mesh.resize(2 * nV);
	for (int i = 0; i < boundary_num; ++i)
	{
		auto v_h = mesh->to_vertex_handle(he_start);
		position_of_mesh(v_h.idx()) = area_1_factor * cos(i * delta_angle);
		position_of_mesh(v_h.idx() + nV) = area_1_factor * sin(-i * delta_angle);
		he_start = mesh->next_halfedge_handle(he_start);
	}

	vector<int> pardiso_it;
	vector<int> pardiso_jt;
	vector<double> pardiso_t;
	vector<double> pardiso_tu;
	vector<double> pardiso_tv;

	pardiso_it.reserve(nV + 1);
	pardiso_jt.reserve(6 * nV);
	pardiso_t.reserve(6 * nV);
	pardiso_tu.resize(nV, 0.0);
	pardiso_tv.resize(nV, 0.0);
	for (size_t i = 0; i < nV; i++)
	{
		pardiso_it.push_back(pardiso_jt.size());

		auto v_h = mesh->vertex_handle(i);
		if (mesh->is_boundary(v_h))
		{
			pardiso_jt.push_back(i);
			pardiso_t.push_back(1);

			pardiso_tu[i] = position_of_mesh(i);
			pardiso_tv[i] = position_of_mesh(i + nV);

		}
		else
		{
			pardiso_jt.push_back(i);
			pardiso_t.push_back(mesh->valence(v_h));
			vector<int> row_id;
			row_id.reserve(mesh->valence(v_h));
			double bu = 0.0; double bv = 0.0;
			for (auto it2 : mesh->vv_range(v_h))
			{
				int vv_id = it2.idx();
				if (mesh->is_boundary(it2))
				{
					bu += position_of_mesh(vv_id);
					bv += position_of_mesh(vv_id + nV);
				}
				else
				{
					if (vv_id > i)
					{
						row_id.push_back(vv_id);
					}
				}
			}
			sort(row_id.begin(), row_id.end(), less<int>());
			for (size_t j = 0; j < row_id.size(); j++)
			{
				pardiso_jt.push_back(row_id[j]);
				pardiso_t.push_back(-1);
			}
			pardiso_tu[i] = bu;
			pardiso_tv[i] = bv;
		}
	}
	pardiso_it.push_back(pardiso_jt.size());

	if (pardiso != NULL)
	{
		delete pardiso;
		pardiso = NULL;
	}
	pardiso = new PardisoSolver();
	pardiso->ia = pardiso_it;
	pardiso->ja = pardiso_jt;
	pardiso->nnz = pardiso_jt.size();
	pardiso->num = nV;

	pardiso->pardiso_init();

	pardiso->a = pardiso_t;

	pardiso->rhs = pardiso_tu;
	pardiso->factorize();


	pardiso->pardiso_solver();

	for (size_t i = 0; i < nV; i++)
	{
		position_of_mesh(i) = (pardiso->result)[i];
	}

	pardiso->rhs = pardiso_tv;
	pardiso->pardiso_solver();
	for (size_t i = 0; i < nV; i++)
	{
		position_of_mesh(i + nV) = (pardiso->result)[i];
	}

	delete pardiso;
	pardiso = NULL;
}

void Progressive::BPE()
{
	cout << "b001" << endl;
	if (pardiso != NULL)
	{
		delete pardiso;
		pardiso = NULL;
	}
	pardiso = new PardisoSolver();
	pardiso->ia = pardiso_ia;
	pardiso->ja = pardiso_ja;
	pardiso->a.resize(pardiso_ja.size());
	pardiso->nnz = pardiso_ja.size();
	pardiso->num = 2 * nV;

	pardiso->pardiso_init();
	cout << "b002" << endl;

	vector<double> energy_area_process;
	energy_area_process.reserve(MAX_ITER_NUM);
	Energysource();

	energy_area_process.push_back(energy_area);
	double energy_pre = 0;
	double energy_cur = energy_uniform;
	
	int iter_num_cur = 0;
	Intp_T_Min = 0;
	changetocm_flag = 0;

	int slim_iter_num = 0;
	int sum_iter_num = 0;

	double conv_percent = 1;

	g_norm=1.0;

	long time_beg, time_end;
	time_beg = clock();
	cout << "b003" << endl;
	while (iter_num_cur < MAX_ITER_NUM)
	{
		iter_num_cur++;
		energy_pre = energy_cur;
		Update_source_same_t();
		if (changetocm_flag < 0.99&&conv_percent>0.1&&Intp_T_Min<0.999)
		{
			SLIM();
			energy_area_process.push_back(energy_area);
			slim_iter_num++;
			sum_iter_num++;

			energy_cur = energy_uniform;
			conv_percent = abs(energy_cur - energy_pre) / energy_pre;
			calc_gradient_norm(position_of_mesh);
			if (conv_percent <= convgence_con_rate||g_norm<= convgence_con_rate)
			{
				break;
			}
		}
		else
		{
			break;
		}
	}
	cout << "b004" << endl;
	int cm_iter_num = 0;
	while (iter_num_cur < MAX_ITER_NUM)
	{
		iter_num_cur++;
		energy_pre = energy_cur;
		Update_source_same_t();
		if (conv_percent > 0.01&&Intp_T_Min<0.999)
		{
			CM();
			energy_area_process.push_back(energy_area);
			cm_iter_num++;
			sum_iter_num++;

			energy_cur = energy_uniform;
			conv_percent = abs(energy_cur - energy_pre) / energy_pre;
			calc_gradient_norm(position_of_mesh);
			if (conv_percent <= convgence_con_rate || g_norm <= convgence_con_rate)
			{
				break;
			}
		}
		else
		{
			recover_to_src();
			energy_cur = energy_area;
			while (iter_num_cur < MAX_ITER_NUM)
			{
				iter_num_cur++;
				energy_pre = energy_cur;
				
				CM();
				energy_area_process.push_back(energy_area);
				sum_iter_num++;

				energy_cur = energy_area;
				conv_percent = abs(energy_cur - energy_pre) / energy_pre;
				calc_gradient_norm(position_of_mesh);
				if (conv_percent <= convgence_con_rate || g_norm <= convgence_con_rate)
				{
					break;
				}
			}
			break;
		}

	}
	cout << "b005" << endl;
	finish_status = true;
	for (size_t i = 0; i < nV; i++)
	{
		para_map(0, i) = position_of_mesh[i];
		para_map(1, i) = position_of_mesh[i + nV];
	}

	time_end = clock();
	time_consumption = (time_end - time_beg) / 1000.0;
	cout << "b006" << endl;
	cout << "PP ====== time_consumption: " << time_consumption << " s;slim_iter: " << slim_iter_num << "; cm_iter: " << cm_iter_num << "; sum_iter: " << sum_iter_num << endl;

	for (size_t i = 0; i < energy_area_process.size(); i++)
	{
		cout <<fixed<<setprecision(8)<<energy_area_process[i]<< endl;
	}

	delete pardiso;
	pardiso = NULL;
	cout << "b007" << endl;
}

void Progressive::recover_to_src()
{
	area = area_src;
	update_p00 = source_p00;
	update_p01 = source_p01;
	update_p10 = source_p10;
	update_p11 = source_p11;
}

void Progressive::Update_source_same_t()
{
	double t_min = 1;
	int geqK = 0;

	vector<double> all_s0(nF);
	vector<double> all_s1(nF);

	vector<double> all_w00(nF);
	vector<double> all_w01(nF);
	vector<double> all_w10(nF);
	vector<double> all_w11(nF);


	int f0, f1, f2;
	double x0, y0, x1, y1, x2, y2;
	double det;
	double E_d;
	double tt;
	double new_sig0, new_sig1;
	double j00, j01, j10, j11;
	double p00, p01, p10, p11;
	double q00, q01, q10, q11;

	double *position = position_of_mesh.data();

	for (int i = 0; i < nF; ++i)
	{
		f0 = tri_vert[i][0];
		f1 = tri_vert[i][1];
		f2 = tri_vert[i][2];

		x0 = position[f0];
		y0 = position[f0 + nV];

		x1 = position[f1];
		y1 = position[f1 + nV];

		x2 = position[f2];
		y2 = position[f2 + nV];

		q00 = x1 - x0; q01 = x2 - x0;
		q10 = y1 - y0; q11 = y2 - y0;

		p00 = source_p00[i]; p01 = source_p01[i]; p10 = source_p10[i]; p11 = source_p11[i];

		j00 = p00*q00 + p10*q01; j01 = p01*q00 + p11*q01; j10 = p00*q10 + p10*q11; j11 = p01*q10 + p11*q11;


		det = j00*j11 - j01*j10;
		E_d = (1 + 1 / (det*det)) * (j00*j00 + j01*j01 + j10*j10 + j11*j11);

		double alpha_0 = j00 + j11; double alpha_1 = j10 - j01;
		double beta_0 = j00 - j11; double beta_1 = j10 + j01;

		double alpha_norm = 0.5*sqrt(alpha_0*alpha_0 + alpha_1*alpha_1);
		double beta_norm = 0.5*sqrt(beta_0*beta_0 + beta_1*beta_1);

		double sig0 = alpha_norm + beta_norm;
		double sig1 = alpha_norm - beta_norm;
		all_s0[i] = sig0;
		all_s1[i] = sig1;

		if (beta_norm < 1e-15)
		{
			all_w00[i] = 0.0;
			all_w01[i] = 0.0;
			all_w10[i] = 0.0;
			all_w11[i] = 0.0;
		}
		else
		{
			double temp = 1 / (sig1*sig1 - sig0 * sig0);
			all_w00[i] = temp * (j00*j00 + j10 * j10 - 0.5*(sig0*sig0 + sig1 * sig1));
			all_w01[i] = temp * (j00*j01 + j10 * j11);
			all_w10[i] = temp * (j01*j00 + j11 * j10);
			all_w11[i] = temp * (j01*j01 + j11 * j11 - 0.5*(sig0*sig0 + sig1 * sig1));
		}

		if (E_d<=bound_distortion_K)
		{
			geqK++;
		}
		else
		{
			tt = newton_equation(sig0, sig1, bound_distortion_K);
			if (tt < t_min)
			{
				t_min = tt;
			}
		}
	}

	changetocm_flag = (double)geqK / nF;

	for (int i = 0; i < nF; ++i)
	{
		double sig0 = all_s0[i];
		double sig1 = all_s1[i];

		new_sig0 = pow(sig0, t_min - 1);
		new_sig1 = pow(sig1, t_min - 1);

		double delta_new = new_sig1 - new_sig0;
		double plus_new = 0.5*(new_sig1 + new_sig0);

		double w00 = delta_new*all_w00[i] + plus_new;
		double w01 = delta_new*all_w01[i];
		double w10 = delta_new*all_w10[i];
		double w11 = delta_new*all_w11[i] + plus_new;

		p00 = source_p00[i]; p01 = source_p01[i]; p10 = source_p10[i]; p11 = source_p11[i];

		update_p00[i] = p00*w00 + p01*w10;
		update_p01[i] = p00*w01 + p01*w11;
		update_p10[i] = p10*w00 + p11*w10;
		update_p11[i] = p10*w01 + p11*w11;
	}

	Intp_T_Min = t_min;
}

void Progressive::SLIM()
{
	double area_now;
	int f0, f1, f2;
	double j00, j01, j10, j11;
	double p00, p01, p10, p11;
	double q00, q01, q10, q11;

	double x0, y0, x1, y1, x2, y2;

	double alpha_norm, beta_norm;

	double alpha_0, alpha_1, beta_0, beta_1;

	double sig0, sig1;

	double det, tr;
	double r0, r1, r2, r3;
	double d00, d01, d02,
		d10, d11, d12;

	double new_sig0, new_sig1;
	double temp;
	double w00, w01, w10, w11;
	double p1, p2, p3, w1, w2, w3;

	double h00, h01, h02, h03, h04, h05,
		h11, h12, h13, h14, h15,
		h22, h23, h24, h25,
		h33, h34, h35,
		h44, h45,
		h55;
	double *position = position_of_mesh.data();

	int nnz = pardiso_ja.size();
	pardiso_a.clear(); pardiso_b.clear();
	pardiso_a.resize(nnz, 0.0);
	pardiso_b.resize(2 * nV, 0.0);

	for (int i = 0; i < nF; ++i)
	{
		area_now = area[i];
		f0 = tri_vert[i][0];
		f1 = tri_vert[i][1];
		f2 = tri_vert[i][2];


		x0 = position[f0];
		y0 = position[f0 + nV];

		x1 = position[f1];
		y1 = position[f1 + nV];

		x2 = position[f2];
		y2 = position[f2 + nV];

		q00 = x1 - x0; q01 = x2 - x0;
		q10 = y1 - y0; q11 = y2 - y0;

		p00 = update_p00[i]; p01 = update_p01[i]; p10 = update_p10[i]; p11 = update_p11[i];

		j00 = p00*q00 + p10*q01; j01 = p01*q00 + p11*q01; j10 = p00*q10 + p10*q11; j11 = p01*q10 + p11*q11;

		alpha_0 = j00 + j11; alpha_1 = j10 - j01;
		beta_0 = j00 - j11; beta_1 = j10 + j01;

		alpha_norm = 0.5*sqrt(alpha_0*alpha_0 + alpha_1*alpha_1);
		beta_norm = 0.5*sqrt(beta_0*beta_0 + beta_1*beta_1);

		sig0 = alpha_norm + beta_norm;
		sig1 = alpha_norm - beta_norm;

		new_sig0 = sqrt(1 + 1 / sig0 + 1 / (sig0*sig0) + 1 / (sig0*sig0*sig0)); new_sig1 = sqrt(1 + 1 / sig1 + 1 / (sig1*sig1) + 1 / (sig1*sig1*sig1));

		if (beta_norm < 1e-15)
		{
			temp = 0;
		}
		else
		{
			temp = (new_sig1 - new_sig0) / (sig1*sig1 - sig0 * sig0);
		}

		w00 = temp*(j00*j00 + j01*j01 - 0.5*(sig0*sig0 + sig1*sig1)) + 0.5*(new_sig0 + new_sig1);
		w01 = temp*(j00*j10 + j01*j11);
		w10 = temp*(j10*j00 + j11*j01);
		w11 = temp*(j10*j10 + j11*j11 - 0.5*(sig0*sig0 + sig1*sig1)) + 0.5*(new_sig0 + new_sig1);

		p1 = p00*p00 + p01*p01; p2 = p00*p10 + p01*p11; p3 = p10*p10 + p11*p11;
		w1 = w00*w00 + w10*w10; w2 = w00*w01 + w10*w11; w3 = w01*w01 + w11*w11;

		area_now *= 2;

		h00 = area_now *(p1 + p2 + p2 + p3)*w1; h01 = -area_now *(p1 + p2)*w1; h02 = -area_now *(p2 + p3)*w1; h03 = area_now *(p1 + p2 + p2 + p3)*w2; h04 = -area_now *(p1 + p2)*w2; h05 = -area_now *(p2 + p3)*w2;
		h11 = area_now *p1*w1;                  h12 = area_now *p2*w1;    	 h13 = -area_now *(p1 + p2)*w2; h14 = area_now *p1*w2;                  h15 = area_now *p2*w2;
		h22 = area_now *p3*w1;                  h23 = -area_now *(p2 + p3)*w2; h24 = area_now *p2*w2;         h25 = area_now *p3*w2;
		h33 = area_now *(p1 + p2 + p2 + p3)*w3; h34 = -area_now *(p1 + p2)*w3; h35 = -area_now *(p2 + p3)*w3;
		h44 = area_now *p1*w3;                  h45 = area_now *p2*w3;
		h55 = area_now *p3*w3;


		det = j00*j11 - j01*j10;
		tr = (j00*j00 + j01*j01 + j10*j10 + j11*j11);

		d00 = -p00 - p10; d01 = p00; d02 = p10;
		d10 = -p01 - p11; d11 = p01; d12 = p11;

		r0 = area_now * ((1 + 1 / (det*det))*j00 - tr*j11 / (det*det*det));
		r1 = area_now * ((1 + 1 / (det*det))*j01 + tr*j10 / (det*det*det));
		r2 = area_now * ((1 + 1 / (det*det))*j10 + tr*j01 / (det*det*det));
		r3 = area_now * ((1 + 1 / (det*det))*j11 - tr*j00 / (det*det*det));


		pardiso_b[f0] -= r0*d00 + r1*d10;
		pardiso_b[f1] -= r0*d01 + r1*d11;
		pardiso_b[f2] -= r0*d02 + r1*d12;
		pardiso_b[f0 + nV] -= r2*d00 + r3*d10;
		pardiso_b[f1 + nV] -= r2*d01 + r3*d11;
		pardiso_b[f2 + nV] -= r2*d02 + r3*d12;

		pardiso_a[id_h00[i]] += h00; pardiso_a[id_h01[i]] += h01; pardiso_a[id_h02[i]] += h02; pardiso_a[id_h03[i]] += h03; pardiso_a[id_h04[i]] += h04; pardiso_a[id_h05[i]] += h05;
		pardiso_a[id_h11[i]] += h11; pardiso_a[id_h12[i]] += h12; pardiso_a[id_h13[i]] += h13; pardiso_a[id_h14[i]] += h14; pardiso_a[id_h15[i]] += h15;
		pardiso_a[id_h22[i]] += h22; pardiso_a[id_h23[i]] += h23; pardiso_a[id_h24[i]] += h24; pardiso_a[id_h25[i]] += h25;
		pardiso_a[id_h33[i]] += h33; pardiso_a[id_h34[i]] += h34; pardiso_a[id_h35[i]] += h35;
		pardiso_a[id_h44[i]] += h44; pardiso_a[id_h45[i]] += h45;
		pardiso_a[id_h55[i]] += h55;

	}

	pardiso->a = pardiso_a;
	pardiso->rhs = pardiso_b;

	pardiso->factorize();
	pardiso->pardiso_solver();

	vector<double> result_d = pardiso->result;

	VectorXd negative_grad(2 * nV), d(2 * nV);
	for (int i = 0; i < 2 * nV; i++)
	{
		negative_grad(i) = pardiso_b[i];
		d(i) = result_d[i];
	}

	double temp_t;
	max_step(position_of_mesh, d, temp_t);

	double alpha = min(1.0, 0.8 * temp_t);
	backtracking_line_search(position_of_mesh, d, negative_grad, alpha);
	position_of_mesh += alpha * d;

	Energysource();
}

void Progressive::CM()
{
	double area_now;
	int f0, f1, f2;
	double j00, j01, j10, j11;
	double p00, p01, p10, p11;
	double q00, q01, q10, q11;

	double x0, y0, x1, y1, x2, y2;

	double hi_0, hi_1;

	double alpha_0, alpha_1, beta_0, beta_1;

	double s1, s2, sig0, sig1;

	double alpha_norm, beta_norm;
	double h_u, h_v, walpha, wbeta;

	double a1x0, a1x1, a1x2, a1x3, a1x4, a1x5,
		a2x0, a2x1, a2x2, a2x3, a2x4, a2x5;

	double aa, bb;
	double uu, vv, uv;
	double u, v;

	double h00, h01, h02, h03, h04, h05,
		h11, h12, h13, h14, h15,
		h22, h23, h24, h25,
		h33, h34, h35,
		h44, h45,
		h55;
	double *position = position_of_mesh.data();
	int nnz = pardiso_ja.size();
	pardiso_a.clear(); pardiso_b.clear();
	pardiso_a.resize(nnz, 0.0);
	pardiso_b.resize(2 * nV, 0.0);

	for (int i = 0; i < nF; ++i)
	{
		area_now = area[i];
		f0 = tri_vert[i][0];
		f1 = tri_vert[i][1];
		f2 = tri_vert[i][2];

		x0 = position[f0];
		y0 = position[f0 + nV];

		x1 = position[f1];
		y1 = position[f1 + nV];

		x2 = position[f2];
		y2 = position[f2 + nV];

		q00 = x1 - x0; q01 = x2 - x0;
		q10 = y1 - y0; q11 = y2 - y0;

		p00 = update_p00[i]; p01 = update_p01[i]; p10 = update_p10[i]; p11 = update_p11[i];

		j00 = p00*q00 + p10*q01; j01 = p01*q00 + p11*q01; j10 = p00*q10 + p10*q11; j11 = p01*q10 + p11*q11;

		alpha_0 = j00 + j11; alpha_1 = j10 - j01;
		beta_0 = j00 - j11;  beta_1 = j10 + j01;

		alpha_norm = 0.5*sqrt(alpha_0*alpha_0 + alpha_1*alpha_1);
		beta_norm = 0.5*sqrt(beta_0*beta_0 + beta_1*beta_1);

		s1 = (p00)*(p00 + p10) + (p01)*(p01 + p11);
		s2 = (p10)*(p00 + p10) + (p11)*(p01 + p11);

		double h1 = p00*p00 + p01*p01;
		double h2 = p00*p10 + p01*p11;
		double h3 = p10*p10 + p11*p11;
		double h4 = p00*p11 - p01*p10;

		a1x0 = alpha_0*(-p00 - p10) + alpha_1*(p01 + p11);  a1x1 = alpha_0*p00 - alpha_1*p01; a1x2 = alpha_0*p10 - alpha_1*p11;
		a1x3 = alpha_0*(-p01 - p11) + alpha_1*(-p00 - p10); a1x4 = alpha_0*p01 + alpha_1*p00; a1x5 = alpha_0*p11 + alpha_1*p10;

		a2x0 = beta_0*(-p00 - p10) + beta_1*(-p01 - p11);   a2x1 = beta_0*p00 + beta_1*p01;   a2x2 = beta_0*p10 + beta_1*p11;
		a2x3 = beta_0*(p01 + p11) + beta_1*(-p00 - p10);    a2x4 = -beta_0*p01 + beta_1*p00;  a2x5 = -beta_0*p11 + beta_1*p10;

		sig0 = alpha_norm + beta_norm;
		sig1 = alpha_norm - beta_norm;

		hi_0 = 2 + 6 * 1 / (sig0*sig0*sig0*sig0); hi_1 = 2 + 6 * 1 / (sig1*sig1*sig1*sig1);

		aa = 0.25 / alpha_norm; bb = 0.25 / beta_norm;

		uu = aa*aa*(area_now*hi_0 + area_now*hi_1);
		vv = bb*bb*(area_now*hi_0 + area_now*hi_1);
		uv = aa*bb*(area_now*hi_0 - area_now*hi_1);

		h_u = area_now * (2 * sig0 - 2 * 1 / (sig0*sig0*sig0));
		h_v = area_now * (2 * sig1 - 2 * 1 / (sig1*sig1*sig1));

		walpha = h_u + h_v;
		wbeta = h_u - h_v;

		double hwa1 = (walpha * 0.25 / alpha_norm); double hwa2 = -(walpha * 0.25*0.25 / (alpha_norm*alpha_norm*alpha_norm));
		double hwb1 = (wbeta * 0.25 / beta_norm); double hwb2 = -(wbeta *0.25*0.25 / (beta_norm*beta_norm*beta_norm));


		h00 = uu*a1x0*a1x0 + vv*a2x0*a2x0 + uv*a1x0*a2x0 + uv*a2x0*a1x0; h01 = uu*a1x0*a1x1 + vv*a2x0*a2x1 + uv*a1x0*a2x1 + uv*a2x0*a1x1; h02 = uu*a1x0*a1x2 + vv*a2x0*a2x2 + uv*a1x0*a2x2 + uv*a2x0*a1x2; h03 = uu*a1x0*a1x3 + vv*a2x0*a2x3 + uv*a1x0*a2x3 + uv*a2x0*a1x3; h04 = uu*a1x0*a1x4 + vv*a2x0*a2x4 + uv*a1x0*a2x4 + uv*a2x0*a1x4; h05 = uu*a1x0*a1x5 + vv*a2x0*a2x5 + uv*a1x0*a2x5 + uv*a2x0*a1x5;

		h11 = uu*a1x1*a1x1 + vv*a2x1*a2x1 + uv*a1x1*a2x1 + uv*a2x1*a1x1; h12 = uu*a1x1*a1x2 + vv*a2x1*a2x2 + uv*a1x1*a2x2 + uv*a2x1*a1x2; h13 = uu*a1x1*a1x3 + vv*a2x1*a2x3 + uv*a1x1*a2x3 + uv*a2x1*a1x3; h14 = uu*a1x1*a1x4 + vv*a2x1*a2x4 + uv*a1x1*a2x4 + uv*a2x1*a1x4; h15 = uu*a1x1*a1x5 + vv*a2x1*a2x5 + uv*a1x1*a2x5 + uv*a2x1*a1x5;

		h22 = uu*a1x2*a1x2 + vv*a2x2*a2x2 + uv*a1x2*a2x2 + uv*a2x2*a1x2; h23 = uu*a1x2*a1x3 + vv*a2x2*a2x3 + uv*a1x2*a2x3 + uv*a2x2*a1x3; h24 = uu*a1x2*a1x4 + vv*a2x2*a2x4 + uv*a1x2*a2x4 + uv*a2x2*a1x4; h25 = uu*a1x2*a1x5 + vv*a2x2*a2x5 + uv*a1x2*a2x5 + uv*a2x2*a1x5;

		h33 = uu*a1x3*a1x3 + vv*a2x3*a2x3 + uv*a1x3*a2x3 + uv*a2x3*a1x3; h34 = uu*a1x3*a1x4 + vv*a2x3*a2x4 + uv*a1x3*a2x4 + uv*a2x3*a1x4; h35 = uu*a1x3*a1x5 + vv*a2x3*a2x5 + uv*a1x3*a2x5 + uv*a2x3*a1x5;

		h44 = uu*a1x4*a1x4 + vv*a2x4*a2x4 + uv*a1x4*a2x4 + uv*a2x4*a1x4; h45 = uu*a1x4*a1x5 + vv*a2x4*a2x5 + uv*a1x4*a2x5 + uv*a2x4*a1x5;

		h55 = uu*a1x5*a1x5 + vv*a2x5*a2x5 + uv*a1x5*a2x5 + uv*a2x5*a1x5;

		if (walpha >= 0)
		{
			h00 += hwa1*(s1 + s2) + hwa2*a1x0*a1x0; h01 += hwa1*(-s1) + hwa2*a1x0*a1x1; h02 += hwa1*(-s2) + hwa2*a1x0*a1x2; h03 += hwa2*a1x0*a1x3; h04 += hwa1*(h4)+hwa2*a1x0*a1x4; h05 += hwa1*(-h4) + hwa2*a1x0*a1x5;
			h11 += hwa1*(h1)+hwa2*a1x1*a1x1;        h12 += hwa1*(h2)+hwa2*a1x1*a1x2;    h13 += hwa1*(-h4) + hwa2*a1x1*a1x3; h14 += hwa2*a1x1*a1x4; h15 += hwa1*(h4)+hwa2*a1x1*a1x5;
			h22 += hwa1*(h3)+hwa2*a1x2*a1x2;        h23 += hwa1*(h4)+hwa2*a1x2*a1x3;    h24 += hwa1*(-h4) + hwa2*a1x2*a1x4; h25 += hwa2*a1x2*a1x5;
			h33 += hwa1*(s1 + s2) + hwa2*a1x3*a1x3; h34 += hwa1*(-s1) + hwa2*a1x3*a1x4; h35 += hwa1*(-s2) + hwa2*a1x3*a1x5;
			h44 += hwa1*(h1)+hwa2*a1x4*a1x4;        h45 += hwa1*(h2)+hwa2*a1x4*a1x5;
			h55 += hwa1*(h3)+hwa2*a1x5*a1x5;

		}
		h00 += hwb1*(s1 + s2) + hwb2*a2x0*a2x0; h01 += hwb1*(-s1) + hwb2*a2x0*a2x1; h02 += hwb1*(-s2) + hwb2*a2x0*a2x2; h03 += hwb2*a2x0*a2x3; h04 += hwb1*(-h4) + hwb2*a2x0*a2x4; h05 += hwb1*(h4)+hwb2*a2x0*a2x5;
		h11 += hwb1*(h1)+hwb2*a2x1*a2x1;        h12 += hwb1*(h2)+hwb2*a2x1*a2x2;    h13 += hwb1*(h4)+hwb2*a2x1*a2x3;    h14 += hwb2*a2x1*a2x4; h15 += hwb1*(-h4) + hwb2*a2x1*a2x5;
		h22 += hwb1*(h3)+hwb2*a2x2*a2x2;        h23 += hwb1*(-h4) + hwb2*a2x2*a2x3; h24 += hwb1*(h4)+hwb2*a2x2*a2x4;    h25 += hwb2*a2x2*a2x5;
		h33 += hwb1*(s1 + s2) + hwb2*a2x3*a2x3; h34 += hwb1*(-s1) + hwb2*a2x3*a2x4; h35 += hwb1*(-s2) + hwb2*a2x3*a2x5;
		h44 += hwb1*(h1)+hwb2*a2x4*a2x4;        h45 += hwb1*(h2)+hwb2*a2x4*a2x5;
		h55 += hwb1*(h3)+hwb2*a2x5*a2x5;

		u = aa*walpha; v = bb*wbeta;

		pardiso_b[f0] -= (u*a1x0 + v*a2x0);
		pardiso_b[f1] -= (u*a1x1 + v*a2x1);
		pardiso_b[f2] -= (u*a1x2 + v*a2x2);
		pardiso_b[f0 + nV] -= (u*a1x3 + v*a2x3);
		pardiso_b[f1 + nV] -= (u*a1x4 + v*a2x4);
		pardiso_b[f2 + nV] -= (u*a1x5 + v*a2x5);

		pardiso_a[id_h00[i]] += h00; pardiso_a[id_h01[i]] += h01; pardiso_a[id_h02[i]] += h02; pardiso_a[id_h03[i]] += h03; pardiso_a[id_h04[i]] += h04; pardiso_a[id_h05[i]] += h05;
		pardiso_a[id_h11[i]] += h11; pardiso_a[id_h12[i]] += h12; pardiso_a[id_h13[i]] += h13; pardiso_a[id_h14[i]] += h14; pardiso_a[id_h15[i]] += h15;
		pardiso_a[id_h22[i]] += h22; pardiso_a[id_h23[i]] += h23; pardiso_a[id_h24[i]] += h24; pardiso_a[id_h25[i]] += h25;
		pardiso_a[id_h33[i]] += h33; pardiso_a[id_h34[i]] += h34; pardiso_a[id_h35[i]] += h35;
		pardiso_a[id_h44[i]] += h44; pardiso_a[id_h45[i]] += h45;
		pardiso_a[id_h55[i]] += h55;
	}

	pardiso->a = pardiso_a;
	pardiso->rhs = pardiso_b;

	pardiso->factorize();
	pardiso->pardiso_solver();

	vector<double> result_d = pardiso->result;

	VectorXd negative_grad(2 * nV), d(2 * nV);
	for (int i = 0; i < 2 * nV; i++)
	{
		negative_grad(i) = pardiso_b[i];
		d(i) = result_d[i];
	}

	pardiso->free_numerical_factorization_memory();

	double temp_t;
	max_step(position_of_mesh, d, temp_t);

	double alpha = 0.95 * temp_t;

	backtracking_line_search(position_of_mesh, d, negative_grad, alpha);

	double e1;
	double s;
	position_of_mesh += alpha * d;
	Energysource();
}

void Progressive::max_step(const VectorXd &xx, const VectorXd &dd, double &step)
{
	double temp_t = numeric_limits<double>::infinity();
	int f0, f1, f2;
	double a, b, c, b1, b2, tt, tt1, tt2;
	double x0, x1, x2, x3, x4, x5, d0, d1, d2, d3, d4, d5;
	const double *x = xx.data();
	const double *d = dd.data();
	for (int i = 0; i < nF; ++i)
	{
		f0 = tri_vert[i][0];
		f1 = tri_vert[i][1];
		f2 = tri_vert[i][2];

		x0 = x[f0]; x1 = x[f1]; x2 = x[f2]; x3 = x[f0 + nV]; x4 = x[f1 + nV]; x5 = x[f2 + nV];
		d0 = d[f0]; d1 = d[f1]; d2 = d[f2]; d3 = d[f0 + nV]; d4 = d[f1 + nV]; d5 = d[f2 + nV];

		a = (d1 - d0) * (d5 - d3) - (d4 - d3) * (d2 - d0);
		b1 = (d1 - d0) * (x5 - x3) + (x1 - x0) * (d5 - d3);
		b2 = (x4 - x3) * (d2 - d0) + (x2 - x0) * (d4 - d3);
		b = b1 - b2;
		c = (x1 - x0) * (x5 - x3) - (x4 - x3) * (x2 - x0);
		tt = get_smallest_pos_quad_zero( a, b, c);
		if (temp_t > tt)
		{
			temp_t = tt;
		}

	}
	if(temp_t==INFINITY)
		temp_t=100;
	step = temp_t;
}

// from libigl
double get_smallest_pos_quad_zero(double a, double b, double c)
{
	using namespace std;
	double t1, t2;
	if (std::abs(a) > 1.0e-10)
	{
		double delta_in = pow(b, 2) - 4 * a * c;
		if (delta_in <= 0)
		{
			return INFINITY;
		}

		double delta = sqrt(delta_in); // delta >= 0
		if (b >= 0) // avoid subtracting two similar numbers
		{
			double bd = -b - delta;
			t1 = 2 * c / bd;
			t2 = bd / (2 * a);
		}
		else
		{
			double bd = -b + delta;
			t1 = bd / (2 * a);
			t2 = (2 * c) / bd;
		}

		assert(std::isfinite(t1));
		assert(std::isfinite(t2));

		if (a < 0) std::swap(t1, t2); // make t1 > t2
		// return the smaller positive root if it exists, otherwise return infinity
		if (t1 > 0)
		{
			return t2 > 0 ? t2 : t1;
		}
		else
		{
			return INFINITY;
		}
	}
	else
	{
		if (b == 0) return INFINITY; // just to avoid divide-by-zero
		t1 = -c / b;
		return t1 > 0 ? t1 : INFINITY;
	}
}
void Progressive::calc_gradient_norm(const VectorXd &x)
{
	double area_now;
	int f0, f1, f2;
	double j00, j01, j10, j11;
	double p00, p01, p10, p11;
	double q00, q01, q10, q11;

	double x0, y0, x1, y1, x2, y2;

	double det, tr;
	double r0, r1, r2, r3;
	double d00, d01, d02,
		d10, d11, d12;

	negative_grad_norm.setZero();

	const double *position = x.data();

	for (int i = 0; i < nF; ++i)
	{
		area_now = area[i];
		f0 = tri_vert[i][0];
		f1 = tri_vert[i][1];
		f2 = tri_vert[i][2];

		x0 = position[f0];
		y0 = position[f0 + nV];

		x1 = position[f1];
		y1 = position[f1 + nV];

		x2 = position[f2];
		y2 = position[f2 + nV];

		q00 = x1 - x0; q01 = x2 - x0;
		q10 = y1 - y0; q11 = y2 - y0;

		p00 = source_p00[i]; p01 = source_p01[i]; p10 = source_p10[i]; p11 = source_p11[i];

		j00 = p00*q00 + p10*q01; j01 = p01*q00 + p11*q01; j10 = p00*q10 + p10*q11; j11 = p01*q10 + p11*q11;

		det = j00*j11 - j01*j10;
		tr = (j00*j00 + j01*j01 + j10*j10 + j11*j11);

		d00 = -p00 - p10; d01 = p00; d02 = p10;
		d10 = -p01 - p11; d11 = p01; d12 = p11;

		r0 = 2 * area_now * ((1 + 1 / (det*det))*j00 - tr*j11 / (det*det*det));
		r1 = 2 * area_now * ((1 + 1 / (det*det))*j01 + tr*j10 / (det*det*det));
		r2 = 2 * area_now * ((1 + 1 / (det*det))*j10 + tr*j01 / (det*det*det));
		r3 = 2 * area_now * ((1 + 1 / (det*det))*j11 - tr*j00 / (det*det*det));

		negative_grad_norm(f0) -= r0*d00 + r1*d10;
		negative_grad_norm(f1) -= r0*d01 + r1*d11;
		negative_grad_norm(f2) -= r0*d02 + r1*d12;
		negative_grad_norm(f0 + nV) -= r2*d00 + r3*d10;
		negative_grad_norm(f1 + nV) -= r2*d01 + r3*d11;
		negative_grad_norm(f2 + nV) -= r2*d02 + r3*d12;
	}

	g_norm = negative_grad_norm.norm();
}

void Progressive::backtracking_line_search(const VectorXd &x, const VectorXd &d, const VectorXd &negetive_grad, double &alpha)
{
	double h = 0.5;
	double tt = -(negetive_grad.transpose()*d)(0, 0);
	double c = 0.2; 
	double ex;
	Energy(x, ex);
	double e;
	VectorXd x_new = x + alpha * d;
	Energy(x_new, e);
	while (e > ex + alpha * c * tt)
	{
		alpha = h*alpha;
		x_new = x + alpha * d;
		Energy(x_new, e);
	}
}

void Progressive::Energy(const VectorXd &position, double &energyupdate)
{
	double energy = 0;


	int f0, f1, f2;
	double x0, y0, x1, y1, x2, y2;
	double det, E_d;
	double j00, j01, j10, j11;
	double p00, p01, p10, p11;
	double q00, q01, q10, q11;
	const double *pos = position.data();
	for (int i = 0; i < nF; ++i)
	{
		f0 = tri_vert[i][0];
		f1 = tri_vert[i][1];
		f2 = tri_vert[i][2];

		x0 = pos[f0];
		y0 = pos[f0 + nV];

		x1 = pos[f1];
		y1 = pos[f1 + nV];

		x2 = pos[f2];
		y2 = pos[f2 + nV];

		q00 = x1 - x0; q01 = x2 - x0;
		q10 = y1 - y0; q11 = y2 - y0;

		p00 = update_p00[i]; p01 = update_p01[i]; p10 = update_p10[i]; p11 = update_p11[i];

		j00 = p00*q00 + p10*q01; j01 = p01*q00 + p11*q01; j10 = p00*q10 + p10*q11; j11 = p01*q10 + p11*q11;


		det = j00*j11 - j01*j10;
		E_d = (1 + 1 / (det*det)) * (j00*j00 + j01*j01 + j10*j10 + j11*j11);

		energy += area[i] * E_d;
	}
	energyupdate = energy;
}

void Progressive::Energysource()
{
	double end_e_one_temp = 0, end_e_area = 0;

	int f0, f1, f2;
	double x0, y0, x1, y1, x2, y2;
	double det, E_1, E_2;

	double j00, j01, j10, j11;
	double p00, p01, p10, p11;
	double q00, q01, q10, q11;

	const double *pos = position_of_mesh.data();
	for (int i = 0; i < nF; ++i)
	{
		f0 = tri_vert[i][0];
		f1 = tri_vert[i][1];
		f2 = tri_vert[i][2];

		x0 = pos[f0];
		y0 = pos[f0 + nV];

		x1 = pos[f1];
		y1 = pos[f1 + nV];

		x2 = pos[f2];
		y2 = pos[f2 + nV];

		q00 = x1 - x0; q01 = x2 - x0;
		q10 = y1 - y0; q11 = y2 - y0;

		p00 = source_p00[i]; p01 = source_p01[i]; p10 = source_p10[i]; p11 = source_p11[i];

		j00 = p00*q00 + p10*q01; j01 = p01*q00 + p11*q01; j10 = p00*q10 + p10*q11; j11 = p01*q10 + p11*q11;

		det = j00*j11 - j01*j10;

		E_1 = (j00*j00 + j01*j01 + j10*j10 + j11*j11);
		E_2 = 1.0 / (det*det)* E_1;

		end_e_one_temp += E_1;
		end_e_one_temp += E_2;
		end_e_area += ((E_1 + E_2)*area_src[i]);
	}
	energy_uniform = end_e_one_temp /nF;

	energy_area = end_e_area;
}


double Progressive::newton_equation(const double & a, const double & b, const double & K)
{
	double tt = 1;
	double E_d = pow(a, 2 * tt) + pow(b, 2 * tt) + pow(1 / a, 2 * tt) + pow(1 / b, 2 * tt) - K;
	while (abs(E_d) > 1e-5)
	{
		tt = tt - 1 / (2 * log(a)*pow(a, 2 * tt) + 2 * log(b)* pow(b, 2 * tt) + 2 * log(1 / a)* pow(1 / a, 2 * tt) + 2 * log(1 / b)*pow(1 / b, 2 * tt))*(pow(a, 2 * tt) + pow(b, 2 * tt) + pow(1 / a, 2 * tt) + pow(1 / b, 2 * tt) - K);
		E_d = pow(a, 2 * tt) + pow(b, 2 * tt) + pow(1 / a, 2 * tt) + pow(1 / b, 2 * tt) - K;
	}
	return tt;
}
