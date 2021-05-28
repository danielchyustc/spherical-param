#include <Engine/MeshEdit/Parameterization/PlanarEmbedding/NoFlipPipeline.h>
#include <Engine/Primitive/TriMesh.h>
#include <gsl/gsl_poly.h>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SVD> 

using namespace Ubpa;

using namespace std;
using namespace Eigen;

bool NoFlipPipeline::Run() {
	if (heMesh->IsEmpty() || !triMesh)
	{
		printf("ERROR::Minsurf::Run\n"
			"\t""heMesh->IsEmpty() || !triMesh\n");
		return false;
	}

	InitPara();
	iter_count_ = 0;
	init_status_ = true;
	finish_status_ = false;

	UpdateTriMesh();
	return true;
}

void NoFlipPipeline::InitPara()
{
	position.resize(2 * nV);
	velocity.resize(2 * nV);
	para_map_.resize(nV, 2);
	step = 0;
	distort_weight.resize(nT);
	velocity.setZero();
	source_local_coordinate_inverse();
	cout << "4";
	Tutte();
	cout << "5";
	gradient.resize(2 * nV);
	ref_lci.resize(nT, 4);
	lci.resize(nT, 4);
	update_reference();
	cout << "6";
	lci = ref_lci;
	sym_dir_energy();
	cout << "7" << endl;
}

void NoFlipPipeline::source_local_coordinate_inverse()
{
	Vector3d e01, e02, n_, x_, y_;
	double x00, x01, x11;
	source_lci.resize(nT, 4);
	for (size_t t = 0; t < nT; t++)
	{
		e01 = input_mesh_pos.col(tri_vert[t][1]) - input_mesh_pos.col(tri_vert[t][0]);
		e02 = input_mesh_pos.col(tri_vert[t][2]) - input_mesh_pos.col(tri_vert[t][0]);
		n_ = e01.cross(e02).normalized();
		x_ = e01.normalized();
		y_ = n_.cross(x_);

		x00 = e01.norm();
		x01 = e02.dot(x_);
		x11 = e02.dot(y_);

		source_lci(t, 0) = 1.0 / x00;	source_lci(t, 1) = -x01 / (x00 * x11);
		source_lci(t, 2) = 0;			source_lci(t, 3) = 1.0 / x11;
	}
}

void NoFlipPipeline::update_reference()
{
	double x0, x1, x2, y0, y1, y2;
	double a, b, c, d;
	Matrix2d P, Q, J, U, V, S;
	double tr, det, D, sig1, sig2;
	Vector2d sigma;
	for (size_t t = 0; t < nT; t++)
	{
		x0 = position(tri_vert[t][0]);	y0 = position(tri_vert[t][0] + nV);
		x1 = position(tri_vert[t][1]);	y1 = position(tri_vert[t][1] + nV);
		x2 = position(tri_vert[t][2]);	y2 = position(tri_vert[t][2] + nV);

		P << source_lci(t, 0), source_lci(t, 1),
			source_lci(t, 2), source_lci(t, 3);
		Q << x1 - x0, x2 - x0,
			y1 - y0, y2 - y0;
		J = Q * P;

		det = J.determinant(); tr = J.squaredNorm();
		D = tr * (1.0 + 1.0 / (det * det));

	//	if (D < 240)
	//	{
	//		ref_lci.row(t) = source_lci.row(t);
	//	}
	//	else
		{
			JacobiSVD svd(J, ComputeFullU | ComputeFullV);
			U = svd.matrixU(); V = svd.matrixV();
			sigma = svd.singularValues();
			sig1 = sigma[0]; sig2 = sigma[1];
			solve_distortion(sig1, sig2);

			S << sig1, 0,
				0, sig2;
			J = U * S * V.transpose();
			P = Q.inverse() * J;

			ref_lci(t, 0) = P(0, 0);	ref_lci(t, 1) = P(0, 1);
			ref_lci(t, 2) = P(1, 0);	ref_lci(t, 3) = P(1, 1);
		}
	}
}

void NoFlipPipeline::solve_distortion(double& sig1, double& sig2)
{
	double s1, s2, D, t = 1, dD_dt, K;
	s1 = pow(sig1, 2 * t); s2 = pow(sig2, 2 * t);
	D = (s1 + s2) * (1.0 + 1.0 / (s1 * s2));
	K = sqrt(D) * 0.8 + 2.4;
	D -= K;
	while (fabs(D) > 1e-5)
	{
		dD_dt = 2 * log(sig1) * (s1 - 1.0 / s1) + 2 * log(sig2) * (s2 - 1.0 / s2);
		t -= D / dD_dt;
		s1 = pow(sig1, 2 * t); s2 = pow(sig2, 2 * t);
		D = (s1 + s2) * (1.0 + 1.0 / (s1 * s2)) - K;
	} 
	sig1 = pow(sig1, t); sig2 = pow(sig2, t);
}

void NoFlipPipeline::Tutte()
{
	SparseMatrix<double, RowMajor> A(nV, nV), zero(1, nV);
	MatrixXd B(nV, 2);
	// construct matrix A, B
	for (size_t i = 0; i < nV; i++)
	{
		auto vi = heMesh->Vertices().at(i);
		A.insert(i, i) = 0;
		B.row(i) = Vector2d::Zero();
		for (auto vj : vi->AdjVertices())
		{
			A.coeffRef(i, i) += 1;
			A.insert(i, heMesh->Index(vj)) = -1;
		}
	}
	zero.setZero();
	B.setZero();

	// set boundary to given value
	size_t nB = heMesh->Boundaries()[0].size();
	double area_coeff = 1 / sqrt(3.1415926);
	for (size_t k = 0; k < nB; k++)
	{
		auto vi = heMesh->Boundaries()[0][k]->Origin();
		size_t i = heMesh->Index(vi);
		A.row(i) = zero;
		A.coeffRef(i, i) = 1;
		B(i, 0) = cos(k * 2 * 3.1415926 / nB) * area_coeff;
		B(i, 1) = -sin(k * 2 * 3.1415926 / nB) * area_coeff;
	}
	A.makeCompressed();

	// solve sparse linear equations
	SparseLU<SparseMatrix<double>> solver;
	solver.compute(A);
	para_map_ = solver.solve(B);

	for (size_t i = 0; i < nV; i++)
	{
		position(i) = para_map_(i, 0);
		position(i + nV) = para_map_(i, 1);
	}
}

bool NoFlipPipeline::Iterate()
{
	if (iter_count_ >= 10000)
	{
		cout << "Have reached maximal iteration number!" << endl;
		return true;
	}
	if (finish_status_)
	{
		cout << "Already finished!" << endl;
		return true;
	}

	for (size_t i = 0; i < 10; i++)
	{
		compute_gradient();
		velocity = -gradient;
		solve_step();
		position += velocity * step;

		double energy_pre = energy;
		sym_dir_energy();
		if (fabs(energy - energy_pre) / energy < 1e-6)
			finish_status_ = true;

		update_reference();
		lci = ref_lci;
	}

	iter_count_++;

	cout << iter_count_ << "th iteration done!" << endl;
	for (size_t i = 0; i < nV; i++)
	{
		para_map_(i, 0) = position(i);
		para_map_(i, 1) = position(i + nV);
	}
	UpdateTriMesh();
	return true;
}

void NoFlipPipeline::compute_distort_weight()
{

}

void NoFlipPipeline::compute_gradient()
{
	double x0, x1, x2, y0, y1, y2;
	double a, b, c, d;
	Matrix2d P, Q, J, J_T;
	double tr, det, K1, K2;
	MatrixXd dJ_dx(2, 3), local_gradient(2, 3);
	gradient.setZero();
	for (size_t t = 0; t < nT; t++)
	{
		x0 = position(tri_vert[t][0]);	y0 = position(tri_vert[t][0] + nV);
		x1 = position(tri_vert[t][1]);	y1 = position(tri_vert[t][1] + nV);
		x2 = position(tri_vert[t][2]);	y2 = position(tri_vert[t][2] + nV);

		P << lci(t, 0), lci(t, 1),
			lci(t, 2), lci(t, 3);
		Q << x1 - x0, x2 - x0,
			y1 - y0, y2 - y0;
		J = Q * P;
		dJ_dx << -(P(0, 0) + P(1, 0)), P(0, 0), P(1, 0),
			-(P(0, 1) + P(1, 1)), P(0, 1), P(1, 1);

		det = J.determinant(); tr = J.squaredNorm();
		a = J(0, 0); b = J(0, 1); c = J(1, 0); d = J(1, 1);
		K1 = 2 * (1.0 + 1.0 / (det * det));
		K2 = 2 * tr / (det * det * det);

		J_T << d, -c,
			-b, a;
		local_gradient = area_weight[t] * (K1 * J - K2 * J_T) * dJ_dx;

		for (size_t k = 0; k < 3; k++)
		{
			gradient[tri_vert[t][k]] += local_gradient(0, k);
			gradient[tri_vert[t][k] + nV] += local_gradient(1, k);
		}
	}
	cout << gradient.norm() << "\t";
}

void NoFlipPipeline::solve_velocity()
{
	velocity = solver.solve(-gradient);
	filter(velocity);
	cout << velocity.norm() << "\t";
}

void NoFlipPipeline::solve_step()
{
	max_step();
	step = min(1.0, 0.8 * step);
	backtracking_line_search();
	cout << step << "\t" << energy << endl;
}

void NoFlipPipeline::sym_dir_energy()
{
	return sym_dir_energy(position, energy);
}

void NoFlipPipeline::sym_dir_energy(VectorXd pos, double& e)
{
	double x0, x1, x2, y0, y1, y2;
	Matrix2d P, Q, J;
	double tr, det, distort;
	e = 0;
	for (size_t t = 0; t < nT; t++)
	{
		x0 = pos(tri_vert[t][0]);	y0 = pos(tri_vert[t][0] + nV);
		x1 = pos(tri_vert[t][1]);	y1 = pos(tri_vert[t][1] + nV);
		x2 = pos(tri_vert[t][2]);	y2 = pos(tri_vert[t][2] + nV);

		P << source_lci(t, 0), source_lci(t, 1),
			source_lci(t, 2), source_lci(t, 3);
		Q << x1 - x0, x2 - x0,
			y1 - y0, y2 - y0;
		J = Q * P;

		det = J.determinant();
		tr = J.squaredNorm();

		distort = area_weight[t] * (1.0 + 1.0 / (det * det)) * tr;
		distort_weight[t] = distort;
		e += distort;
	}
	distort /= e;
}


void NoFlipPipeline::max_step()
{
	step = numeric_limits<double>::infinity();
	int f0, f1, f2;
	double a, b, c, b1, b2, step_t, r1, r2;
	double x0, x1, x2, x3, x4, x5, d0, d1, d2, d3, d4, d5;
	const double* x = position.data();
	const double* d = velocity.data();
	for (size_t t = 0; t < nT; t++)
	{
		f0 = tri_vert[t][0];
		f1 = tri_vert[t][1];
		f2 = tri_vert[t][2];

		x0 = x[f0]; x1 = x[f1]; x2 = x[f2]; x3 = x[f0 + nV]; x4 = x[f1 + nV]; x5 = x[f2 + nV];
		d0 = d[f0]; d1 = d[f1]; d2 = d[f2]; d3 = d[f0 + nV]; d4 = d[f1 + nV]; d5 = d[f2 + nV];

		a = (d1 - d0) * (d5 - d3) - (d4 - d3) * (d2 - d0);
		b1 = (d1 - d0) * (x5 - x3) + (x1 - x0) * (d5 - d3);
		b2 = (x4 - x3) * (d2 - d0) + (x2 - x0) * (d4 - d3);
		b = b1 - b2;
		c = (x1 - x0) * (x5 - x3) - (x4 - x3) * (x2 - x0);
		step_t = numeric_limits<double>::infinity();
		if (gsl_poly_solve_quadratic(a, b, c, &r1, &r2) == 2)
			if (r1 > 0) step_t = r1;
			else if (r1 < 0 && r2 > 0) step_t = r2;
		if (step > step_t) step = step_t;
	}
}

void NoFlipPipeline::backtracking_line_search()
{
	/*
	double tt = gradient.dot(velocity);
	double ex, e;
	sym_dir_energy(position, ex);
	VectorXd x_new = position + step * velocity;
	sym_dir_energy(x_new, e);
	while (e > ex + step * 0.2 * tt)
	{
		step *= 0.5;
		x_new = position + step * velocity;
		sym_dir_energy(x_new, e);
	}*/
	double t_start = 0, t_end = step;
	double t_delta, t_temp;
	VectorXd x_now(2 * nV);
	double e_min = energy, e[11];
	int k = 0;
	for (size_t i = 0; i < 5; i++)
	{
		t_delta = (t_end - t_start) / 10;
		for (size_t j = 0; j <= 10; j++)
		{
			t_temp = t_start + j * t_delta;
			x_now = position + velocity * t_temp;
			sym_dir_energy(x_now, e[j]);
			if (e_min > e[j])
			{
				e_min = e[j];
				k = j;
			}
		}
		if (k == 0) t_end = t_start + t_delta;
		else if (k == 10) t_start = t_end - t_delta;
		else if (e[k - 1] < e[k + 1])
		{
			t_start += (k - 1) * t_delta;
			t_end = t_start + t_delta;
		}
		else
		{
			t_start += k * t_delta;
			t_end = t_start + t_delta;
		}
	}
	step = t_start;
}

void NoFlipPipeline::UpdateTriMesh()
{
	// half-edge structure -> triangle mesh
	vector<pointf3> positions;
	vector<unsigned> indice;
	vector<normalf> normals = triMesh->GetNormals();
	vector<pointf2> texcoords;
	positions.reserve(nV);
	indice.reserve(3 * nT);
	texcoords.reserve(nV);

	double min_x = para_map_(0, 0);
	double min_y = para_map_(0, 1);
	double max_x = para_map_(0, 0);
	double max_y = para_map_(0, 1);

	for (size_t i = 0; i < nV; i++)
	{
		double x = para_map_(i, 0);
		double y = para_map_(i, 1);
		if (x < min_x)
			min_x = x;
		else if (x > max_x)
			max_x = x;

		if (y < min_y)
			min_y = y;
		else if (y > max_y)
			max_y = y;
	}

	for (size_t i = 0; i < nV; i++)
	{
		switch (view_mode_)
		{
		case kPlane:
			positions.push_back({ para_map_(i, 0), para_map_(i, 1), 0 });
			break;
		case kMesh:
			positions.push_back(heMesh->Vertices().at(i)->pos.cast_to<pointf3>());
			break;
		default:
			break;
		}
		double x = (para_map_(i, 0) - min_x) / (max_x - min_x);
		double y = (para_map_(i, 1) - min_y) / (max_y - min_y);
		texcoords.push_back({ x, y });
	}
	for (auto f : heMesh->Polygons())
	{
		for (auto v : f->BoundaryVertice())
		{
			indice.push_back(static_cast<unsigned>(heMesh->Index(v)));
		}
	}

	triMesh->Init(indice, positions, normals, texcoords);
}

void NoFlipPipeline::compute_solver_AQP()
{
	SparseMatrix<double, RowMajor> A(nV, nV);
	BuildCotanWeightMatrix();
	A = CotW;
	A.coeffRef(0, 0) += 1;
	A.makeCompressed();
	solver.compute(A);
}

void NoFlipPipeline::compute_solver_AKVF()
{
	SparseMatrix<double, RowMajor> A(2 * nV, 2 * nV);
	int f0, f1, f2;
	double x0, x1, x2, y0, y1, y2;
	double p00, p01, p10, p11;
	Matrix2d P, Q;
	VectorXd J[6];
	VectorXi index(6);
	A.setIdentity();
	A /= 10.0 * nT;

	for (size_t i = 0; i < 6; i++) J[i].resize(4);

	for (size_t t = 0; t < nV; t++)
	{
		f0 = tri_vert[t][0]; f1 = tri_vert[t][1]; f2 = tri_vert[t][2];
		index << f0, f1, f2, f0 + nV, f1 + nV, f2 + nV;

		x0 = position(f0);	y0 = position(f0 + nV);
		x1 = position(f1);	y1 = position(f1 + nV);
		x2 = position(f2);	y2 = position(f2 + nV);

		Q << x1 - x0, x2 - x0,
			y1 - y0, y2 - y0;
		P = Q.inverse();
		
		p00 = P(0, 0); p01 = P(0, 1); p10 = P(1, 0); p11 = P(1, 1);
		J[0] << -2 * (p00 + p10), -p01 - p11, -p01 - p11, 0;
		J[1] << 2 * p00, p01, p01, 0;
		J[2] << 2 * p10, p11, p11, 0;
		J[3] << 0, -p00 - p10, -p00 - p10, -2 * (p01 + p11);
		J[4] << 0, p00, p00, 2 * p01;
		J[5] << 0, p10, p10, 2 * p11;

		for (size_t i = 0; i < 6; i++)
			for (size_t j = 0; j < 6; j++)
				A.coeffRef(index[i], index[j]) += J[i].dot(J[j]) * area_weight[t];
	}
	A.makeCompressed();
	solver.compute(A);
}

void NoFlipPipeline::compute_solver_SLIM()
{
	SparseMatrix<double, RowMajor> A(2 * nV, 2 * nV);
	int f0, f1, f2;
	double x0, x1, x2, y0, y1, y2;
	double a, b, c, d;
	Matrix2d P, Q, J, U, W;
	double tr, det, sig1, sig2, Sq, Sq1, Sq2, K1, K2;
	MatrixXd dJ_dx(2, 3);
	Matrix3d L;
	Vector3i index;
	A.insert(0, 0) = 1;
	A.insert(nV, nV) = 1;
	for (size_t t = 0; t < nT; t++)
	{
		f0 = tri_vert[t][0];
		f1 = tri_vert[t][1];
		f2 = tri_vert[t][2];

		x0 = position(f0);	y0 = position(f0 + nV);
		x1 = position(f1);	y1 = position(f1 + nV);
		x2 = position(f2);	y2 = position(f2 + nV);

		P << lci(t, 0), lci(t, 1),
			lci(t, 2), lci(t, 3);
		Q << x1 - x0, x2 - x0,
			y1 - y0, y2 - y0;
		J = Q * P;
		dJ_dx << -(P(0, 0) + P(1, 0)), P(0, 0), P(1, 0),
			-(P(0, 1) + P(1, 1)), P(0, 1), P(1, 1);

		det = J.determinant();	tr = J.squaredNorm();
		a = J(0, 0); b = J(0, 1); c = J(1, 0); d = J(1, 1);

		K1 = b * b + d * d - a * a - c * c;
		K2 = 2 * (a * b + c * d);
		Sq1 = sqrt(tr + 2 * det);
		Sq2 = sqrt(tr - 2 * det);
		Sq = Sq1 * Sq2;
		sig1 = (Sq1 - Sq2) / 2;
		sig2 = (Sq1 + Sq2) / 2;
		U << K1 + Sq, K2,
			-K2, K1 + Sq;

		W << (sig1 + 1) * (sig1 * sig1 + 1) / (sig1 * sig1 * sig1), 0, 0, (sig2 + 1)* (sig2 * sig2 + 1) / (sig2 * sig2 * sig2);
		W = U * W * U.transpose() / (2 * Sq * (Sq + K1));
		L = dJ_dx.transpose() * W * dJ_dx;
		L *= iter_count_ == 0 ? 1.0 / nT : area_weight[t];
//		L *= area_weight[t];

		index << f0, f1, f2;
		for (size_t i = 0; i < 3; i++)
		{
			for (size_t j = 0; j < 3; j++)
			{
				A.coeffRef(index[i], index[j]) += L(i, j);
				A.coeffRef(index[i] + nV, index[j] + nV) += L(i, j);
			}
		}
	}
	A.makeCompressed();
	solver.compute(A);
}

void NoFlipPipeline::compute_solver_CM()
{
	SparseMatrix<double, RowMajor> A(2 * nV, 2 * nV);
	int f0, f1, f2;
	double x0, x1, x2, y0, y1, y2;
	Matrix2d P, Q, J, h_hess;
	double tr, det, a, b, c, d, sqa, sqb, u, v;
	Vector2d h_grad;
	VectorXd grada(4), gradb(4), alpha_grad(4), beta_grad(4);
	MatrixXd alpha_hess(4, 4), beta_hess(4, 4), hess_J(4, 4), hessa(4, 4), hessb(4, 4);
	MatrixXd dg_dJ(4, 2), dJ_dx(2, 3);
	MatrixXd local_hessian(6, 6);
	VectorXi index(6);

	hessa <<
		1, 0, 0, 1,
		0, 1, -1, 0,
		0, -1, 1, 0,
		1, 0, 0, 1;
	hessb <<
		1, 0, 0, -1,
		0, 1, 1, 0,
		0, 1, 1, 0,
		-1, 0, 0, 1;

	A.setIdentity();
	A *= 10;

	for (size_t t = 0; t < nV; t++)
	{
		f0 = tri_vert[t][0];
		f1 = tri_vert[t][1];
		f2 = tri_vert[t][2];

		local_hessian.setZero();

		x0 = position(f0);	y0 = position(f0 + nV);
		x1 = position(f1);	y1 = position(f1 + nV);
		x2 = position(f2);	y2 = position(f2 + nV);

		P << lci(t, 0), lci(t, 1),
			lci(t, 2), lci(t, 3);
		Q << x1 - x0, x2 - x0,
			y1 - y0, y2 - y0;
		J = Q * P;
		dJ_dx << -(P(0, 0) + P(1, 0)), P(0, 0), P(1, 0),
			-(P(0, 1) + P(1, 1)), P(0, 1), P(1, 1);

		a = J(0, 0); b = J(0, 1); c = J(1, 0); d = J(1, 1);
		det = a * d - b * c;	tr = a * a + b * b + c * c + d * d;
		sqa = sqrt(tr + 2 * det);	sqb = sqrt(tr - 2 * det);
		u = (sqa + sqb) / 2;		v = (sqa - sqb) / 2;

		grada << a + d, b - c, c - b, a + d;
		gradb << a - d, b + c, b + c, d - a;
		alpha_grad = 0.5 * grada / sqa;
		beta_grad = 0.5 * gradb / sqb;
		dg_dJ.col(0) = alpha_grad + beta_grad;
		dg_dJ.col(1) = alpha_grad - beta_grad;
		h_grad << 2 * u - 2.0 / (u * u * u), 2 * v - 2.0 / (v * v * v);


		h_hess << 2 + 6.0 / (u * u * u * u), 0,
			0, 2 + 6.0 / (v * v * v * v);
		alpha_hess = 0.5 * hessa / sqa - 0.5 * grada * grada.transpose() / (sqa * sqa * sqa);
		beta_hess = 0.5 * hessb / sqb - 0.5 * gradb * gradb.transpose() / (sqb * sqb * sqb);

		hess_J = dg_dJ * h_hess * dg_dJ.transpose() + (h_grad[0] - h_grad[1]) * beta_hess;
		if(h_grad[0] + h_grad[1] > 0) hess_J += (h_grad[0] + h_grad[1]) * alpha_hess;

		local_hessian.block(0, 0, 3, 3) = dJ_dx.transpose() * hess_J.block(0, 0, 2, 2) * dJ_dx;
		local_hessian.block(0, 3, 3, 3) = dJ_dx.transpose() * hess_J.block(0, 2, 2, 2) * dJ_dx;
		local_hessian.block(3, 0, 3, 3) = local_hessian.block(0, 3, 3, 3).transpose();
		local_hessian.block(3, 3, 3, 3) = dJ_dx.transpose() * hess_J.block(2, 2, 2, 2) * dJ_dx;
		local_hessian *= area_weight[t];

		index << f0, f1, f2, f0 + nV, f1 + nV, f2 + nV;
		for (size_t i = 0; i < 6; i++)
			for (size_t j = 0; j < 6; j++)
				A.coeffRef(index[i], index[j]) += local_hessian(i, j);
	}
	A.makeCompressed();
	solver.compute(A);
}


void NoFlipPipeline::filter(VectorXd vec)
{
	double x_cen = 0, y_cen = 0;
	VectorXd rot_vec(2 * nV);
	for (size_t i = 0; i < nV; i++)
	{
		x_cen += vec[i];
		y_cen += vec[i + nV];
		rot_vec[i] = position[i + nV];
		rot_vec[i + nV] = -position[i];
	}
	for (size_t i = 0; i < nV; i++)
	{
		vec[i] -= x_cen;
		vec[i + nV] -= y_cen;
	}
	rot_vec.normalize();
	vec -= vec.dot(rot_vec) * rot_vec;
}
