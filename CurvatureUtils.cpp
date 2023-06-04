// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2023 Crane Chen <cranechen7@gmail.com>
// SPDX-License-Identifier: MPL-2.0
// ----------------------------------------------------------------------------

#include "open3d/Open3D.h"
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <omp.h>

namespace open3d {
namespace geometry {


  std::vector<std::set<int>> CurvatureUtils::VertexFaceAdjacency(const Eigen::MatrixXi &F) {
      std::vector<std::set<int>> adjacency(F.maxCoeff() + 1);

      for (int i = 0; i < F.rows(); ++i) {
          for (int j = 0; j < 3; ++j) {
              int vertex_index = F(i, j);
              adjacency[vertex_index].insert(i);
          }
      }

      return adjacency;
  }


  Eigen::SparseMatrix<double> CurvatureUtils::CotangentLaplacian(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
      int n = V.rows();
      Eigen::SparseMatrix<double> L(n, n);

      std::vector<Eigen::Triplet<double>> triplets;
      triplets.reserve(12 * F.rows());

      Eigen::VectorXd diagonalEntries = Eigen::VectorXd::Zero(n);

      for (int i = 0; i < F.rows(); ++i) {
          for (int j = 0; j < 3; ++j) {
              int v1 = F(i, j);
              int v2 = F(i, (j + 1) % 3);
              int v3 = F(i, (j + 2) % 3);

              Eigen::RowVector3d p1 = V.row(v1);
              Eigen::RowVector3d p2 = V.row(v2);
              Eigen::RowVector3d p3 = V.row(v3);

              Eigen::RowVector3d u = p1 - p3;
              Eigen::RowVector3d v = p2 - p3;

              double cotangent = u.dot(v) / u.cross(v).norm();

              triplets.push_back(Eigen::Triplet<double>(v1, v2, 0.5 * cotangent));
              triplets.push_back(Eigen::Triplet<double>(v2, v1, 0.5 * cotangent));
              diagonalEntries(v1) -= 0.5 * cotangent;
              diagonalEntries(v2) -= 0.5 * cotangent;
          }
      }

      // Add diagonal entries
      for (int i = 0; i < n; ++i) {
          triplets.push_back(Eigen::Triplet<double>(i, i, diagonalEntries(i)));
      }

      L.setFromTriplets(triplets.begin(), triplets.end());

      return L;
  }



  Eigen::MatrixXd CurvatureUtils::ComputeDoubleFaceAreas(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
      Eigen::MatrixXd A(F.rows(), 1);

      for (int i = 0; i < F.rows(); ++i) {
          const Eigen::RowVector3i &triangle = F.row(i);
          const Eigen::RowVector3d &v0 = V.row(triangle(0));
          const Eigen::RowVector3d &v1 = V.row(triangle(1));
          const Eigen::RowVector3d &v2 = V.row(triangle(2));

          double area = (v1 - v0).cross(v2 - v0).norm();
          A(i, 0) = area;
      }

      return A;
  }

}  // namespace geometry
}  // namespace open3d




