#include <Engine/MeshEdit/Parameterization/PlanarEmbedding/GD.h>
#include <Engine/Primitive/TriMesh.h>
#include <gsl/gsl_poly.h>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SVD> 

using namespace Ubpa;

using namespace std;
using namespace Eigen;
