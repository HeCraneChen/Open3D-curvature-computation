// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2023 Crane Chen <cranechen7@gmail.com>
// SPDX-License-Identifier: MPL-2.0
// ----------------------------------------------------------------------------

#include "TotalCurvature.h"
#include "CurvatureUtils.h"
#include <omp.h>

namespace open3d {
namespace geometry {

double TotalCurvature::PerTriangleLaplacianCurvature(
    int row_id,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXd& N,
    const Eigen::MatrixXi& F,
    const Eigen::VectorXd& A) 
{
    Eigen::MatrixXd n_adjacent_face, v_adjacent_face, x, y, z;
    Eigen::MatrixXi f(1, 3), adjacent_faces, adjacent_face;
    Eigen::SparseMatrix<double> l;
    double total_curvature_one_triangle, cx, cy, cz, face_area;
    face_area = *A(row_id, Eigen::all).data();
    face_area = face_area / 2;

    adjacent_face = F(row_id, Eigen::all);
    v_adjacent_face = V({adjacent_face(0), adjacent_face(1), adjacent_face(2)}, Eigen::all);
    n_adjacent_face = N({adjacent_face(0), adjacent_face(1), adjacent_face(2)}, Eigen::all);

    f << 0 , 1 , 2;
    l = CurvatureUtils::CotangentLaplacian(v_adjacent_face, f);
    x = n_adjacent_face(Eigen::all,0);
    y = n_adjacent_face(Eigen::all,1);
    z = n_adjacent_face(Eigen::all,2);
    cx = (x.transpose() * l * x)(0);
    cy = (y.transpose() * l * y)(0);
    cz = (z.transpose() * l * z)(0);
    total_curvature_one_triangle = - cx - cy - cz;

    return total_curvature_one_triangle;
}


void TotalCurvature::TotalCurvatureMesh(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& N,
    Eigen::VectorXd& k_S) 
{
  using namespace Eigen;
  using namespace std;

  Eigen::SparseMatrix<double> G;
  Eigen::MatrixXd A;
  A = CurvatureUtils::ComputeDoubleFaceAreas(V,F);
  auto adjacency = CurvatureUtils::VertexFaceAdjacency(F);
  int V_num = V.rows();
  int F_num = F.rows();
  Eigen::VectorXd k_S_face(F_num);
  k_S.resize(V.rows()); 

  // per-face curvature
  #pragma omp parallel for
  for(int i = 0; i<F_num; i++){
    k_S_face(i) = TotalCurvature::PerTriangleLaplacianCurvature(i, V, N, F, A);
  }
  // deriving per-vertex curvature
  #pragma omp parallel for
  for (int i = 0; i<V_num; i++){
    double total_area = 0;
    double k_S_entry = 0;
    int N_adjacent_faces = 0;
    for (const auto &face : adjacency[i]) {
            total_area += 0.5 * A(face);
            k_S_entry += k_S_face(face);
    }
    k_S(i) = k_S_entry / total_area;
  }
}

}  // namespace geometry
}  // namespace open3d




