// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "open3d/Open3D.h"
#include "TotalCurvature.h"
#include "TotalCurvaturePointCloud.h"
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>

// Function to convert scalar values into a colormap (Jet colormap in this case)
Eigen::Vector3d scalar_to_color(double scalar, double min_val, double max_val) {
    double value = (scalar - min_val) / (max_val - min_val);
    double r = 1.0, g = 1.0, b = 1.0;

    if (value < 0.5) {
        r = value * 2.0;
        g = value * 2.0;
        b = 1.0;
    } else {
        r = 1.0;
        g = 1.0 - (value - 0.5) * 2.0;
        b = 1.0 - (value - 0.5) * 2.0;
    }

    return Eigen::Vector3d(r, g, b);
}

std::shared_ptr<open3d::geometry::PointCloud> ReadPointCloudFromFile(const std::string& filename) {
    // Create an empty PointCloud object
    auto point_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();

    // Read the point cloud from the file
    if (open3d::io::ReadPointCloud(filename, *point_cloud_ptr)) {
        std::cout << "Successfully read the point cloud: " << filename << std::endl;
    } else {
        std::cout << "Failed to read the point cloud: " << filename << std::endl;
        point_cloud_ptr.reset();
    }

    return point_cloud_ptr;
}




int main(int argc, char *argv[]) {
    if (argc == 2) {
        std::string option(argv[1]);
        if (option == "--skip-for-unit-test") {
            open3d::utility::LogInfo("Skiped for unit test.");
            return 0;
        }
    }
    
    ////////////////////////// for triangle mesh
    ////// reading in a triangle mesh and visualize
    std::string filename = "../data/cow.ply";
    // Create an empty TriangleMesh object
    auto mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>();
    // Read the mesh from the file
    if (open3d::io::ReadTriangleMesh(filename, *mesh_ptr)) {
        std::cout << "Successfully read the mesh: " << filename << std::endl;
    } else {
        std::cout << "Failed to read the mesh: " << filename << std::endl;
        mesh_ptr.reset();
    }
    mesh_ptr->ComputeVertexNormals();
    
    Eigen::MatrixXd V, N;
    Eigen::MatrixXi F;
    Eigen::VectorXd k_S;
    open3d::geometry::TriangleMesh mesh = *mesh_ptr;
    V = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(
        reinterpret_cast<const double*>(mesh.vertices_.data()), mesh.vertices_.size(), 3);
    F = Eigen::Map<const Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>>(
        reinterpret_cast<const int*>(mesh.triangles_.data()), mesh.triangles_.size(), 3);
    N = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(
        reinterpret_cast<const double*>(mesh.vertex_normals_.data()), mesh.vertex_normals_.size(), 3);

    // calculate total curvature on triangle mesh
    open3d::geometry::TotalCurvature::TotalCurvatureMesh(V, F, N, k_S);

    // Find the min and max values in scalar_values
    Eigen::VectorXd k_S_vis = k_S.array().pow(0.0425);
    double min_val = k_S_vis.minCoeff();
    double max_val = k_S_vis.maxCoeff();

    // Convert scalar values into colormap and assign to the vertices
    mesh.vertex_colors_.resize(mesh.vertices_.size());
    for (size_t i = 0; i < mesh.vertices_.size(); ++i) {
        mesh.vertex_colors_[i] = scalar_to_color(k_S_vis(i), min_val, max_val);
    }

    // Visualize the mesh
    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::TriangleMesh>(mesh)}, "Mesh with Colormap");
    //////////////////////////

    //////////////////////////
    ////// reading in a point cloud and visualize
    std::string filename_v = "../data/cow_points.ply";
    std::string filename_n = "../data/cow_normals.ply";
    // Create an empty PointCloud object
    auto point_cloud_ptr_v = std::make_shared<open3d::geometry::PointCloud>();
    auto point_cloud_ptr_n = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud(filename_v, *point_cloud_ptr_v);
    open3d::io::ReadPointCloud(filename_n, *point_cloud_ptr_n);

    // Get the vertex matrix V from the point cloud
    std::vector<Eigen::Vector3d> points_v = point_cloud_ptr_v->points_;
    std::vector<Eigen::Vector3d> points_n = point_cloud_ptr_n->points_;
    Eigen::MatrixXd V_PCD(points_v.size(), 3);
    Eigen::MatrixXd N_PCD(points_v.size(), 3);
    for (size_t i = 0; i < points_v.size(); ++i) {
        V_PCD.row(i) = points_v[i];
        N_PCD.row(i) = points_n[i];
    }

    Eigen::VectorXd k_S_PCD(V_PCD.rows());

    // calculate total curvature on point cloud
    open3d::geometry::TotalCurvaturePointCloud::TotalCurvaturePCD(V_PCD, N_PCD, k_S_PCD, 20);

    // Apply the color map to the point cloud
    // Find the min and max values in scalar_values
    Eigen::VectorXd k_S_PCD_vis = k_S_PCD.array().abs().pow(0.0425);
    double min_val_pcd = k_S_PCD_vis.minCoeff();
    double max_val_pcd = k_S_PCD_vis.maxCoeff();
    point_cloud_ptr_v->colors_.resize(point_cloud_ptr_v->points_.size());
    for (int i = 0; i < point_cloud_ptr_v->points_.size(); ++i) {
        point_cloud_ptr_v->colors_[i] = scalar_to_color(k_S_PCD_vis(i), min_val_pcd, max_val_pcd);
    }

    // Visualize the colored point cloud
    auto vis = std::make_shared<open3d::visualization::Visualizer>();
    vis->CreateVisualizerWindow("Colored Point Cloud", 800, 600);
    vis->AddGeometry(point_cloud_ptr_v);
    vis->Run();
    vis->DestroyVisualizerWindow();
    //////////////////////////

    

    return 0;
}
