// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2023 Crane Chen <cranechen7@gmail.com>
// SPDX-License-Identifier: MPL-2.0
// ----------------------------------------------------------------------------

#include "TotalCurvaturePointCloud.h"
#include "CurvatureUtils.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <omp.h>
#include <thread>
#include <sstream>
#include <random>
#include <vector>
#include <algorithm>
#include <utility>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include "open3d/Open3D.h"
#include "CurvatureUtils.h"
#include "TotalCurvaturePointCloud.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K> Gt;
typedef CGAL::Triangulation_vertex_base_with_info_2<int, Gt> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2<Gt, Tds> Delaunay;


namespace open3d {
namespace geometry {

std::vector<int> TotalCurvaturePointCloud::Where(int i, const Eigen::MatrixXi& inArray)
{
    std::vector<int> res;
    int idx = 0;
    for (int r = 0; r < inArray.rows(); r++){
      for (int c = 0; c < inArray.cols(); c++){
        if(inArray(r,c) == i){
          res.push_back(r);
        }
      }
    }
    return res;
}


std::vector<Eigen::Vector3d> TotalCurvaturePointCloud::MatrixToVector3d(const Eigen::MatrixXd& mat) {
    std::vector<Eigen::Vector3d> vec(mat.rows());
    for (int i = 0; i < mat.rows(); ++i) {
        vec[i] = mat.row(i);
    }
    return vec;
}


bool TotalCurvaturePointCloud::compare_triangles(const Eigen::Vector3i& t1, const Eigen::Vector3i& t2) {
    int sum_t1 = t1(0) + t1(1) + t1(2);
    int sum_t2 = t2(0) + t2(1) + t2(2);
    return sum_t1 < sum_t2;
}


Eigen::MatrixXi TotalCurvaturePointCloud::DelaunayKNN(
  const Eigen::MatrixXd& knn_locations_including_self, 
  const Eigen::MatrixXi& idx)
{   
    Eigen::MatrixXi adjacent_triangles_idx_local, adjacent_triangles_idx, faces;
    Eigen::MatrixXd n, A;
    Eigen::Vector3d mean_A_vec, r0, x_axis, r1, n_plane;

    // projecting points from 3d to 2d, find the best fitting plane
    mean_A_vec = knn_locations_including_self.transpose().rowwise().mean();
    A = knn_locations_including_self.transpose().colwise() - mean_A_vec; // 3, 20 
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    n = svd.matrixU()(Eigen::all, svd.matrixU().cols()-1);
    n_plane = {n(0), n(1), n(2)};
    r0 = {knn_locations_including_self(0,0), knn_locations_including_self(0,1), knn_locations_including_self(0,2)};
    x_axis = {1, 0, 0};
    auto e1 = n_plane.cross(x_axis);
    auto e2 = n_plane.cross(e1);
    auto subtracted_r0 = knn_locations_including_self.rowwise() - r0.transpose();
    auto projected_e1 = subtracted_r0 * e1; 
    auto projected_e2 = subtracted_r0 * e2;

    // Delaunay triangulation to find one ring idx
    Eigen::MatrixXd knn_locations_2d(projected_e1.rows(), 2);
    knn_locations_2d << projected_e1, projected_e2; // 20, 2

 
    std::vector<std::pair<K::Point_3, int>> points_with_indices;
    for (size_t i = 0; i < knn_locations_2d.rows(); ++i) {
        points_with_indices.emplace_back(K::Point_3(knn_locations_2d(i, 0), knn_locations_2d(i, 1), 0.0), i);
    }

    Delaunay dt(points_with_indices.begin(), points_with_indices.end());

    // Create an Eigen matrix to store the triangles
    Eigen::MatrixXi all_triangles(dt.number_of_faces(), 3);

    size_t triangle_index = 0;
    for (Delaunay::Finite_faces_iterator face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
        Eigen::Vector3i triangle(face->vertex(0)->info(), face->vertex(1)->info(), face->vertex(2)->info());
        all_triangles.row(triangle_index) = triangle;
        triangle_index++;
    }
 

    std::vector<int> adjacent_triangles_mask = TotalCurvaturePointCloud::Where(0, all_triangles);
    adjacent_triangles_idx_local =  all_triangles(adjacent_triangles_mask, Eigen::all); // 5, 3
    // Converting indices back to original global indices
    Eigen::MatrixXi adjacent_triangles_idx_x = idx(Eigen::all,adjacent_triangles_idx_local(Eigen::all,0));
    Eigen::MatrixXi adjacent_triangles_idx_y = idx(Eigen::all,adjacent_triangles_idx_local(Eigen::all,1));
    Eigen::MatrixXi adjacent_triangles_idx_z = idx(Eigen::all,adjacent_triangles_idx_local(Eigen::all,2));
    Eigen::MatrixXi adjacent_triangles_idx_tmp(3, adjacent_triangles_idx_x.cols());
    adjacent_triangles_idx_tmp << adjacent_triangles_idx_x, adjacent_triangles_idx_y, adjacent_triangles_idx_z;
    adjacent_triangles_idx = adjacent_triangles_idx_tmp.transpose();
    return adjacent_triangles_idx;
}


double TotalCurvaturePointCloud::PerTriangleLaplacianTriangleFanCurvature(
  const Eigen::MatrixXi& adjacent_triangles_idx, 
  const Eigen::MatrixXd& V, 
  const Eigen::MatrixXd& N)
{
    Eigen::MatrixXd triangle, n_triangle_face, v_triangle_fan, n_triangle_fan, x, y, z;
    Eigen::MatrixXi f(1, 3), adjacent_triangle_idx;
    Eigen::SparseMatrix<double> l;
    double cx, cy, cz;
    double total_curvature = 0.0, total_area = 0.0;
    int N_triangles = adjacent_triangles_idx.rows();
    for (int i = 0; i < N_triangles; i++){    
      adjacent_triangle_idx = adjacent_triangles_idx(i,Eigen::all);
      std::vector<int> triangle_verts = {adjacent_triangle_idx(0,0), adjacent_triangle_idx(0,1), adjacent_triangle_idx(0,2)};
      triangle = V(triangle_verts, Eigen::all); //3, 3
      n_triangle_face = N(triangle_verts, Eigen::all); // 3, 3
      Eigen::MatrixXd AB_mat = triangle(0, Eigen::all) - triangle(1, Eigen::all);
      Eigen::MatrixXd AC_mat = triangle(0, Eigen::all) - triangle(2, Eigen::all);
      Eigen::Vector3d AB(AB_mat.coeff(0, 0), AB_mat.coeff(0, 1), AB_mat.coeff(0, 2));
      Eigen::Vector3d AC(AC_mat.coeff(0, 0), AC_mat.coeff(0, 1), AC_mat.coeff(0, 2));
      double triangle_area = 0.5 * sqrt((AB.cross(AC)).squaredNorm());
      f << 0 , 1 , 2;
      l = CurvatureUtils::CotangentLaplacian(triangle, f);
      x = n_triangle_face(Eigen::all,0);
      y = n_triangle_face(Eigen::all,1);
      z = n_triangle_face(Eigen::all,2);
      cx = (x.transpose() * l * x)(0);
      cy = (y.transpose() * l * y)(0);
      cz = (z.transpose() * l * z)(0); 
      total_curvature += (- cx - cy - cz);
      total_area += triangle_area;
    }
    total_curvature = total_curvature / total_area;
    return total_curvature;
}


void TotalCurvaturePointCloud::TotalCurvaturePCD(
  const Eigen::MatrixXd& V, 
  const Eigen::MatrixXd& N, 
  Eigen::VectorXd& k_S, 
  int K)
{

  // Convert the Eigen matrix to an Open3D PointCloud
  open3d::geometry::PointCloud point_cloud;
  point_cloud.points_ = TotalCurvaturePointCloud::MatrixToVector3d(V);

  // Build the KDTree
  std::cout<<"building kdtree..."<<std::endl;
  open3d::geometry::KDTreeFlann kdtree;
  kdtree.SetGeometry(point_cloud);
  std::cout<<"successfully built kdtree!"<<std::endl;

  // Curvature Estimation
  std::cout<<"calculating total curvature..."<<std::endl;
  // #pragma omp parallel for
  for (int i =0; i<V.rows(); i++){
    std::vector<int> idx_vec;
    std::vector<double> distances;
    kdtree.SearchKNN(point_cloud.points_[i], K, idx_vec, distances);
    Eigen::MatrixXd knn_locations_including_self = V(idx_vec,Eigen::all);
    int n_rows = idx_vec.size();
    Eigen::MatrixXi idx = Eigen::MatrixXi::Zero(1, n_rows); // Create a column vector of zeros with the same number of rows as idx_vec
    for (int i = 0; i < n_rows; i++) {
        idx(0, i) = idx_vec[i];
    }
    Eigen::MatrixXi adjacent_triangles_idx = TotalCurvaturePointCloud::DelaunayKNN(knn_locations_including_self, idx);
    k_S(i) = TotalCurvaturePointCloud::PerTriangleLaplacianTriangleFanCurvature(adjacent_triangles_idx, V, N);
  }
}

}  // namespace geometry
}  // namespace open3d















