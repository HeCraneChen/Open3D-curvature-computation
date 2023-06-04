// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2023 Crane Chen <cranechen7@gmail.com>
// SPDX-License-Identifier: MPL-2.0
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <memory>
#include <vector>
#include <set>

namespace open3d {
namespace geometry {

class TriangleMesh;

class CurvatureUtils {
public:
    static std::vector<std::set<int>> VertexFaceAdjacency(const Eigen::MatrixXi &F);

    static Eigen::SparseMatrix<double> CotangentLaplacian(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);

    static Eigen::MatrixXd ComputeDoubleFaceAreas(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
};

}  // namespace geometry
}  // namespace open3d

#ifndef OPEN3D_STATIC_LIBRARY
#  include "CurvatureUtils.cpp"
#endif

