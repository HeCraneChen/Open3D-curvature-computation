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

class TotalCurvaturePointCloud {
public:
    static std::vector<int> Where(int i, const Eigen::MatrixXi& inArray);

    static std::vector<Eigen::Vector3d> MatrixToVector3d(const Eigen::MatrixXd& mat);

    static bool compare_triangles(const Eigen::Vector3i& t1, const Eigen::Vector3i& t2);

    static Eigen::MatrixXi DelaunayKNN(const Eigen::MatrixXd& knn_locations_including_self, const Eigen::MatrixXi& idx);

    static double PerTriangleLaplacianTriangleFanCurvature(const Eigen::MatrixXi& adjacent_triangles_idx, const Eigen::MatrixXd& V, const Eigen::MatrixXd& N);

    static void TotalCurvaturePCD(const Eigen::MatrixXd& V, const Eigen::MatrixXd& N, Eigen::VectorXd& k_S, int K);
};

}  // namespace geometry
}  // namespace open3d

#ifndef OPEN3D_STATIC_LIBRARY
#  include "TotalCurvaturePointCloud.cpp"
#endif

