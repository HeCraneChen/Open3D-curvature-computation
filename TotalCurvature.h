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

namespace open3d {
namespace geometry {

class TriangleMesh;

class TotalCurvature {
public:
    static double PerTriangleLaplacianCurvature(
        int row_id,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXd& N,
        const Eigen::MatrixXi& F,
        const Eigen::VectorXd& A);

    static void TotalCurvatureMesh(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::MatrixXd& N,
        Eigen::VectorXd& k_S);
};

}  // namespace geometry
}  // namespace open3d

#ifndef OPEN3D_STATIC_LIBRARY
#  include "TotalCurvature.cpp"
#endif
